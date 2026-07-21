# Room Segmentation Module

## Overview

This module implements automatic room segmentation from occupancy grids built during robot exploration. It identifies distinct rooms/spaces and tracks them across frames to maintain consistent room IDs throughout navigation.

## Implementation Details

### Core Algorithm

The room segmentation pipeline follows these steps:

1. **Binary Conversion**: Convert occupancy grid to binary format
   - Free space (explored and not obstacle) = 1
   - Obstacles or unexplored = 0

2. **Morphological Closing**: Apply morphological dilation to close small gaps
   - Uses elliptical kernel (size 5x5)
   - Connects nearby free spaces that belong to the same room
   - Handles doorways and small openings appropriately

3. **Connected Components Analysis**: Identify separate connected regions
   - Each connected component represents a potential room
   - Uses `scipy.ndimage.label` for efficient labeling

4. **Small Region Filtering**: Remove debris and small artifacts
   - Default threshold: 5% of total explored area
   - Prevents tiny regions from being classified as rooms

5. **Room Tracking**: Track rooms across frames using IoU matching
   - Computes Intersection over Union (IoU) between current and previous rooms
   - Default IoU threshold: 0.3
   - Maintains consistent room IDs as exploration progresses
   - Handles room appearance/disappearance gracefully

### Files

- **`utils/room_segmentation.py`**: Core room segmentation module
  - `RoomSegmentation` class with all processing logic
  - Room tracking and visualization methods
  
- **`main.py`**: Integration into the main navigation pipeline
  - Initialization before episode loop
  - Reset at episode start
  - Processing after map updates
  - Visualization alongside existing outputs

- **`test_room_segmentation.py`**: Test script with synthetic data
  - Demonstrates room segmentation on synthetic floor plans
  - Tests tracking across frames
  - Generates visualizations

## Usage

### In Main Pipeline

The room segmentation is automatically integrated into the main navigation loop:

```python
# Initialization (before episodes)
room_segmentation = RoomSegmentation(min_room_area_ratio=0.05, iou_threshold=0.3)

# Reset at episode start
room_segmentation.reset()

# Process occupancy grid (during navigation)
room_map, room_info = room_segmentation.process_occupancy_grid(full_map_pred)

# Access room information
for room_id, info in room_info.items():
    print(f"Room {room_id}: area={info['area']}, center={info['center']}")
```

### Standalone Usage

```python
from utils.room_segmentation import RoomSegmentation
import torch

# Initialize
room_seg = RoomSegmentation(
    min_room_area_ratio=0.05,  # Minimum room size (5% of total area)
    iou_threshold=0.3           # IoU threshold for tracking
)

# Process a map
# full_map_pred: Tensor of shape [C, H, W]
#   - Channel 0: obstacle map
#   - Channel 1: explored map
room_map, room_info = room_seg.process_occupancy_grid(full_map_pred)

# Visualize
room_vis = room_seg.visualize_rooms(room_map, room_info)

# Query room at position
room_id = room_seg.get_room_at_position(room_map, x=100, y=200)
```

## Parameters

### RoomSegmentation Constructor

- **`min_room_area_ratio`** (float, default=0.05): Minimum room area as ratio of total explored area
  - Lower values: Detect smaller rooms/spaces
  - Higher values: Only detect large rooms, filter out small spaces

- **`iou_threshold`** (float, default=0.3): IoU threshold for matching rooms across frames
  - Lower values: More strict matching, may create new room IDs more often
  - Higher values: More lenient matching, maintains IDs longer

## Output Format

### `room_map`
- NumPy array of shape `[H, W]` with integer room IDs
- 0 = no room (obstacle or unexplored)
- 1, 2, 3, ... = room IDs

### `room_info`
Dictionary mapping room_id to properties:
```python
{
    room_id: {
        'area': int,              # Room area in pixels
        'center': (float, float), # (x, y) center coordinates
        'mask': np.ndarray        # Binary mask of room region
    },
    ...
}
```

## Visualization

Room visualizations are automatically saved to:
```
{dump_location}/dump/{exp_name}/episodes/eps_{episode_n}/rooms/room_segmentation_{step}.png
```

Each room is colored with a distinct random color, with room IDs and centers marked.

## Testing

Run the test script to verify the implementation:

```bash
cd /home/jl/MCoCoNav
conda activate Mcoconav
python test_room_segmentation.py
```

This will:
1. Create a synthetic 3-room floor plan
2. Apply room segmentation
3. Test tracking across multiple frames
4. Generate visualization showing all processing steps

Output saved to: `room_segmentation_test.png`

## Integration with Navigation

The room segmentation data can be used for:

1. **Hierarchical Planning**: Plan navigation at room level
2. **Task Assignment**: Assign different rooms to different agents
3. **Exploration Strategy**: Prioritize unexplored rooms
4. **Semantic Understanding**: Associate objects with specific rooms
5. **Communication**: Agents can reference rooms by ID in coordination

## Example Log Output

When running the main pipeline, you'll see:

```
=====> Detected 3 rooms: [1, 2, 3]
```

This indicates that 3 distinct rooms have been identified with IDs 1, 2, and 3.

## Future Enhancements

Potential improvements:

- [ ] Room labeling based on semantic content (bedroom, kitchen, etc.)
- [ ] Door detection and connectivity graph between rooms
- [ ] Room boundary refinement using walls
- [ ] Persistent room IDs across episodes (save/load room database)
- [ ] Hierarchical room clustering (sub-rooms within larger spaces)

## Technical Notes

### Performance
- Connected components analysis: O(N) where N = map size
- IoU computation: O(K*M) where K = current rooms, M = historical rooms
- Efficient for real-time operation with typical map sizes (480x480)

### Edge Cases Handled
- Empty maps (no rooms detected)
- Map size changes across frames
- Room merging/splitting during exploration
- Temporary occlusions

### Dependencies
- NumPy: Array operations
- OpenCV (cv2): Morphological operations
- SciPy: Connected components labeling
- PyTorch: Tensor handling (for input compatibility)

## Contact

For questions or issues with room segmentation, please refer to the main project documentation.

