import numpy as np
import cv2
from scipy.ndimage import label, distance_transform_edt
from typing import Dict, List, Tuple


class RoomSegmentation:
    """
    Room segmentation from occupancy grid using morphological operations
    and connected components analysis.
    """
    
    def __init__(self, min_room_area_ratio=0.05, iou_threshold=0.3, max_largest_room_ratio=0.8, min_area_cells=100):
        """
        Args:
            min_room_area_ratio: Minimum room area as ratio of total area (default 5%)
            iou_threshold: IoU threshold for tracking rooms across frames (τ: 0.3~0.5)
            max_largest_room_ratio: Maximum ratio for largest room (default 80%)
            min_area_cells: Minimum area in cells for fragment removal (50~200 cells)
        """
        self.min_room_area_ratio = min_room_area_ratio
        self.iou_threshold = iou_threshold
        self.max_largest_room_ratio = max_largest_room_ratio
        self.min_area_cells = min_area_cells  # 50~200 cells as per requirement
        
        # Store room information across frames
        self.room_history = {}  # room_id -> latest room mask
        self.next_room_id = 1
        self.room_centers = {}  # room_id -> (center_x, center_y)
        
        # Track ID switches for stability metrics
        self.id_switch_history = []  # List of (step, num_switches) tuples
        self.stability_window = 20  # Number of steps to check stability
        self.previous_room_ids = set()  # Track previous step's room IDs
        
    def process_occupancy_grid(self, obstacle_map, explored_map):
        """
        Process occupancy grid to segment rooms based on obstacle map.
        
        Args:
            obstacle_map: numpy array or Tensor of shape [H, W] where:
                - 1 = obstacle, 0 = free space
            explored_map: numpy array or Tensor of shape [H, W] where:
                - 1 = explored, 0 = unexplored
                
        Returns:
            room_map: numpy array of shape [H, W] with room IDs
            room_info: dict mapping room_id to room properties
        """
        # Convert to numpy if needed
        import torch
        if isinstance(obstacle_map, torch.Tensor):
            obstacle_map = obstacle_map.cpu().numpy()
        if isinstance(explored_map, torch.Tensor):
            explored_map = explored_map.cpu().numpy()
        
        # 4.2 基础二值图构建 (Basic binary map construction)
        # free_mask = explored_mask & (occupancy == free)
        free_mask = np.logical_and(
            explored_map > 0.1,  # explored_mask
            obstacle_map < 0.5   # occupancy == free
        ).astype(np.uint8)
        
        # obs_mask = (occupancy == occupied) | (~explored_mask)
        # 关键: unknown 当障碍,避免未知区域把空间连成一大块
        # Key: treat unknown as obstacle to avoid connecting unknown areas into one large block
        obs_mask = np.logical_or(
            obstacle_map >= 0.5,      # occupancy == occupied
            explored_map <= 0.1       # ~explored_mask (unknown as obstacle)
        ).astype(np.uint8)
        
        # 4.3 关门法(Door-closing) + 连通域
        # Step 1: Estimate adaptive dilation radius r using distance transform
        # 做 distance transform D (free 到障碍距离)
        # r = clamp(round(percentile(D, 10)), 2, 10)
        # But use more conservative approach to avoid over-merging rooms
        free_for_dt = free_mask.astype(np.float32)
        if np.sum(free_for_dt) > 0:
            # Distance transform: distance from free cells to obstacles
            # D: distance from each free cell to nearest obstacle
            D = distance_transform_edt(free_for_dt)
            # Get 10th percentile of D (as per requirement)
            if np.sum(D) > 0:
                # Only consider distances from free cells (D > 0)
                D_free = D[D > 0]
                if len(D_free) > 0:
                    percentile_30 = np.percentile(D_free, 30)  # percentile of free cell distances
                    r = int(np.clip(np.round(percentile_30), 2, 100))  # clamp(round(...), 2, 10) as per requirement
                else:
                    r = 5  # Default fallback
            else:
                r = 5  # Default fallback
        else:
            r = 5  # Default fallback
        
        # Step 2: Dilate obstacles with adaptive radius r
        # Separate actual obstacles from unknown areas
        actual_obstacles = (obstacle_map >= 0.5).astype(np.uint8)
        unknown_areas = (explored_map <= 0.1).astype(np.uint8)
        
        # Dilate actual obstacles with radius r (adaptive)
        kernel_size = 2 * r + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        obs_inflated = cv2.dilate(actual_obstacles, kernel, iterations=1)
        
        # For unknown areas, also use adaptive radius r (not fixed small kernel)
        # This ensures consistent behavior - unknown areas treated same as obstacles
        unknown_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        unknown_inflated = cv2.dilate(unknown_areas, unknown_kernel, iterations=1)
        
        # Combine: obstacles OR unknown areas (both dilated with same radius r)
        obs_inflated = np.logical_or(obs_inflated, unknown_inflated).astype(np.uint8)
        
        # Step 3: free_closed = explored_mask & (~obs_inflated)
        # Only mark as free if it's explored AND not in inflated obstacles
        free_closed = np.logical_and(
            explored_map > 0.1,  # explored_mask
            obs_inflated == 0    # ~obs_inflated
        ).astype(np.uint8)
        
        # Step 4: Connected components on free_closed → room_id_raw
        labeled_map, num_components = label(free_closed)
        
        # 4.4 后处理:去碎片 (Post-processing: remove fragments)
        # Calculate total explored area
        total_explored_area = np.sum(explored_map > 0.1)
        min_area_ratio = total_explored_area * self.min_room_area_ratio
        min_area = max(self.min_area_cells, min_area_ratio)  # Use max of ratio-based or cell-based
        
        # Collect all components with their properties
        all_components = []
        for component_id in range(1, num_components + 1):
            component_mask = (labeled_map == component_id)
            component_area = np.sum(component_mask)
            
            all_components.append({
                'component_id': component_id,
                'mask': component_mask,
                'area': component_area
            })
        
        # Separate large rooms from small fragments
        large_rooms = [comp for comp in all_components if comp['area'] >= min_area]
        small_fragments = [comp for comp in all_components if comp['area'] < min_area]
        
        # Merge small fragments to adjacent largest room
        # 对面积 < min_area 的 room: 合并到边界相邻且面积最大的房间
        valid_components = large_rooms.copy()
        
        if len(small_fragments) > 0 and len(large_rooms) > 0:
            # Create a map of large rooms for quick lookup
            large_room_map = np.zeros_like(labeled_map)
            for room in large_rooms:
                large_room_map[room['mask']] = room['component_id']
            
            # For each fragment, find adjacent largest room
            for fragment in small_fragments:
                fragment_mask = fragment['mask']
                
                # Dilate fragment slightly to find adjacent rooms
                frag_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
                frag_dilated = cv2.dilate(fragment_mask.astype(np.uint8), frag_kernel, iterations=1)
                
                # Find adjacent room IDs
                adjacent_room_ids = np.unique(large_room_map[frag_dilated > 0])
                adjacent_room_ids = adjacent_room_ids[adjacent_room_ids > 0]
                
                if len(adjacent_room_ids) > 0:
                    # Merge to largest adjacent room
                    adjacent_rooms = [r for r in large_rooms if r['component_id'] in adjacent_room_ids]
                    if len(adjacent_rooms) > 0:
                        target_room = max(adjacent_rooms, key=lambda x: x['area'])
                        # Merge fragment into target room
                        target_room['mask'] = np.logical_or(target_room['mask'], fragment_mask)
                        target_room['area'] = np.sum(target_room['mask'])
                # If no adjacent room, fragment is discarded (too isolated)
        
        # Check largest room ratio (避免全都一间 - avoid all being one room)
        if len(valid_components) > 0:
            # Sort by area (largest first)
            valid_components.sort(key=lambda x: x['area'], reverse=True)
            largest_area = valid_components[0]['area']
            largest_room_ratio = largest_area / total_explored_area if total_explored_area > 0 else 0
            
            # If largest room is too big (> 80%), reject it (避免全都一间)
            if largest_room_ratio > self.max_largest_room_ratio:
                # Reject the largest room and keep only smaller ones
                # This prevents everything from being classified as one room
                valid_components = [comp for comp in valid_components 
                                   if comp['area'] / total_explored_area <= self.max_largest_room_ratio]
                # If we filtered everything out, keep all except the largest
                if len(valid_components) == 0:
                    # Keep all except the largest room
                    valid_components = valid_components[1:] if len(valid_components) > 1 else []
        
        # Step 5: Track rooms across frames using IoU matching
        room_map, room_info = self._track_rooms(valid_components, total_explored_area)
        
        # Track ID switches for stability metrics
        self._track_id_switches(room_info)
        
        return room_map, room_info
    
    def _track_rooms(self, current_components: List[Dict], total_area: int) -> Tuple[np.ndarray, Dict]:
        """
        Track rooms across frames using IoU matching.
        
        Args:
            current_components: List of component dictionaries with 'mask' and 'area'
            
        Returns:
            room_map: numpy array with tracked room IDs
            room_info: dict with room properties
        """
        if len(current_components) == 0:
            # No valid rooms found
            empty_shape = (480, 480)  # Default shape
            if len(current_components) == 0 and len(self.room_history) > 0:
                # Get shape from history
                first_room = next(iter(self.room_history.values()))
                empty_shape = first_room.shape
            return np.zeros(empty_shape, dtype=np.int32), {}
        
        # Get shape from first component
        map_shape = current_components[0]['mask'].shape
        room_map = np.zeros(map_shape, dtype=np.int32)
        room_info = {}
        
        # If this is the first frame, assign new IDs to all components
        if len(self.room_history) == 0:
            for comp in current_components:
                room_id = self.next_room_id
                self.next_room_id += 1
                
                room_map[comp['mask']] = room_id
                self.room_history[room_id] = comp['mask']
                
                # Calculate center
                ys, xs = np.where(comp['mask'])
                center_y, center_x = np.mean(ys), np.mean(xs)
                self.room_centers[room_id] = (center_x, center_y)
                
                room_info[room_id] = {
                    'area': comp['area'],
                    'center': (center_x, center_y),
                    'mask': comp['mask']
                }
        else:
            # Match current components with historical rooms using IoU
            matched_rooms = set()
            
            for comp in current_components:
                best_iou = 0
                best_room_id = None
                
                # Compare with all historical rooms
                for room_id, hist_mask in self.room_history.items():
                    iou = self._calculate_iou(comp['mask'], hist_mask)
                    
                    if iou > best_iou and iou >= self.iou_threshold:
                        best_iou = iou
                        best_room_id = room_id
                
                # Assign room ID
                if best_room_id is not None and best_room_id not in matched_rooms:
                    # Matched with existing room
                    room_id = best_room_id
                    matched_rooms.add(room_id)
                else:
                    # New room
                    room_id = self.next_room_id
                    self.next_room_id += 1
                
                # Update room map and history
                room_map[comp['mask']] = room_id
                self.room_history[room_id] = comp['mask']
                
                # Calculate center
                ys, xs = np.where(comp['mask'])
                center_y, center_x = np.mean(ys), np.mean(xs)
                self.room_centers[room_id] = (center_x, center_y)
                
                room_info[room_id] = {
                    'area': comp['area'],
                    'center': (center_x, center_y),
                    'mask': comp['mask']
                }
            
            # Remove old rooms that weren't matched (disappeared)
            # Keep history for a few frames to handle temporary occlusions
            rooms_to_keep = set(room_info.keys())
            self.room_history = {k: v for k, v in self.room_history.items() 
                               if k in rooms_to_keep}
            self.room_centers = {k: v for k, v in self.room_centers.items() 
                               if k in rooms_to_keep}
        
        return room_map, room_info
    
    def _calculate_iou(self, mask1: np.ndarray, mask2: np.ndarray) -> float:
        """
        Calculate Intersection over Union (IoU) between two binary masks.
        
        Args:
            mask1: Binary mask 1
            mask2: Binary mask 2
            
        Returns:
            IoU value between 0 and 1
        """
        # Ensure masks have the same shape
        if mask1.shape != mask2.shape:
            # If shapes differ, return 0 IoU (no match)
            return 0.0
        
        intersection = np.logical_and(mask1, mask2).sum()
        union = np.logical_or(mask1, mask2).sum()
        
        if union == 0:
            return 0.0
        
        return intersection / union
    
    def visualize_rooms(self, room_map: np.ndarray, room_info: Dict) -> np.ndarray:
        """
        Create a visualization of the room segmentation.
        
        Args:
            room_map: Room segmentation map with room IDs
            room_info: Dictionary with room information
            
        Returns:
            vis_map: RGB visualization of rooms
        """
        vis_map = np.zeros((*room_map.shape, 3), dtype=np.uint8)
        
        # Generate distinct colors for each room
        np.random.seed(42)  # For reproducible colors
        colors = {}
        
        for room_id in room_info.keys():
            # Generate random color for each room
            colors[room_id] = tuple(np.random.randint(50, 255, 3).tolist())
        
        # Color each room
        for room_id, color in colors.items():
            vis_map[room_map == room_id] = color
        
        # Draw room centers and IDs
        for room_id, info in room_info.items():
            center_x, center_y = info['center']
            center_x, center_y = int(center_x), int(center_y)
            
            # Draw center point
            cv2.circle(vis_map, (center_x, center_y), 3, (255, 255, 255), -1)
            
            # Draw room ID
            cv2.putText(vis_map, f"R{room_id}", (center_x + 5, center_y - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        return vis_map
    
    def get_room_at_position(self, room_map: np.ndarray, x: int, y: int) -> int:
        """
        Get the room ID at a specific position.
        
        Args:
            room_map: Room segmentation map
            x, y: Position coordinates
            
        Returns:
            Room ID at the position (0 if no room)
        """
        if 0 <= y < room_map.shape[0] and 0 <= x < room_map.shape[1]:
            return int(room_map[y, x])
        return 0
    
    def _track_id_switches(self, room_info: Dict):
        """
        Track ID switches for stability metrics.
        Records how many room IDs changed compared to previous step.
        """
        current_room_ids = set(room_info.keys())
        
        if len(self.previous_room_ids) > 0:
            # Count ID switches: rooms that disappeared or new rooms appeared
            disappeared = self.previous_room_ids - current_room_ids
            appeared = current_room_ids - self.previous_room_ids
            
            # ID switch rate = (disappeared + appeared) / max(previous_count, current_count)
            max_count = max(len(self.previous_room_ids), len(current_room_ids))
            if max_count > 0:
                switch_rate = (len(disappeared) + len(appeared)) / max_count
            else:
                switch_rate = 0.0
            
            # Record for history
            self.id_switch_history.append({
                'step': len(self.id_switch_history),
                'switch_rate': switch_rate,
                'disappeared': len(disappeared),
                'appeared': len(appeared),
                'room_ids': current_room_ids
            })
            
            # Keep only last N steps for stability check
            if len(self.id_switch_history) > self.stability_window:
                self.id_switch_history.pop(0)
        
        # Update previous room IDs for next step
        self.previous_room_ids = current_room_ids.copy()
    
    def get_id_switch_rate(self, window_size: int = None) -> float:
        """
        Get average ID switch rate over recent steps.
        
        Args:
            window_size: Number of recent steps to consider (default: stability_window)
            
        Returns:
            Average switch rate (0.0 to 1.0)
        """
        if window_size is None:
            window_size = self.stability_window
        
        if len(self.id_switch_history) == 0:
            return 0.0
        
        recent_history = self.id_switch_history[-window_size:]
        if len(recent_history) == 0:
            return 0.0
        
        avg_switch_rate = np.mean([h['switch_rate'] for h in recent_history])
        return avg_switch_rate
    
    def check_stability(self, window_size: int = None) -> bool:
        """
        Check if room IDs are stable for N consecutive steps.
        According to DoD: 连续 20 帧 room id 颜色基本稳定
        
        Args:
            window_size: Number of steps to check (default: stability_window = 20)
            
        Returns:
            True if stable (switch rate < 5%), False otherwise
        """
        if window_size is None:
            window_size = self.stability_window
        
        if len(self.id_switch_history) < window_size:
            return False  # Not enough history yet
        
        recent_history = self.id_switch_history[-window_size:]
        avg_switch_rate = np.mean([h['switch_rate'] for h in recent_history])
        
        # Stable if switch rate < 5%
        return avg_switch_rate < 0.05
    
    def reset(self):
        """Reset the room tracking history."""
        self.room_history = {}
        self.next_room_id = 1
        self.room_centers = {}
        self.id_switch_history = []
        self.previous_room_ids = set()

