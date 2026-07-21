"""PointCloud2 serialization helpers for RGB-D mapping."""

from __future__ import annotations

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def build_xyzrgb_cloud(
    points: np.ndarray,
    colors_rgb: np.ndarray,
    header: Header,
    pixels_uv: np.ndarray | None = None,
) -> PointCloud2:
    """Build XYZ/RGB and optional source-pixel fields without point loops."""
    point_array = np.asarray(points, dtype=np.float32)
    color_array = np.asarray(colors_rgb, dtype=np.uint8)
    if point_array.ndim != 2 or point_array.shape[1] != 3:
        raise ValueError(f"Points must have shape (N, 3), got {point_array.shape}")
    if color_array.shape != point_array.shape:
        raise ValueError(
            f"RGB colors must have shape {point_array.shape}, got {color_array.shape}"
        )
    pixel_array: np.ndarray | None = None
    if pixels_uv is not None:
        raw_pixels = np.asarray(pixels_uv)
        if raw_pixels.shape != (point_array.shape[0], 2) or not np.issubdtype(
            raw_pixels.dtype, np.integer
        ):
            raise ValueError("pixels_uv must be an integer array with shape (N, 2)")
        if np.any(raw_pixels < 0) or np.any(raw_pixels > np.iinfo(np.uint32).max):
            raise ValueError("pixels_uv values must fit uint32")
        pixel_array = raw_pixels.astype(np.uint32, copy=False)

    packed_rgb = (
        color_array[:, 0].astype(np.uint32) << 16
        | color_array[:, 1].astype(np.uint32) << 8
        | color_array[:, 2].astype(np.uint32)
    )
    fields = [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")]
    if pixel_array is not None:
        fields.extend((("u", "<u4"), ("v", "<u4")))
    cloud_dtype = np.dtype(fields)
    cloud_data = np.empty(point_array.shape[0], dtype=cloud_dtype)
    cloud_data["x"] = point_array[:, 0]
    cloud_data["y"] = point_array[:, 1]
    cloud_data["z"] = point_array[:, 2]
    cloud_data["rgb"] = packed_rgb
    if pixel_array is not None:
        cloud_data["u"] = pixel_array[:, 0]
        cloud_data["v"] = pixel_array[:, 1]

    message = PointCloud2()
    message.header = header
    message.height = 1
    message.width = int(point_array.shape[0])
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
    ]
    if pixel_array is not None:
        message.fields.extend(
            [
                PointField(
                    name="u", offset=16, datatype=PointField.UINT32, count=1
                ),
                PointField(
                    name="v", offset=20, datatype=PointField.UINT32, count=1
                ),
            ]
        )
    message.is_bigendian = False
    message.point_step = cloud_dtype.itemsize
    message.row_step = message.point_step * message.width
    message.is_dense = bool(np.all(np.isfinite(point_array)))
    message.data = cloud_data.tobytes()
    return message


def build_xyz_probability_cloud(
    points: np.ndarray,
    probabilities: np.ndarray,
    header: Header,
) -> PointCloud2:
    """Build an XYZ/occupancy PointCloud2 for occupied voxel centers."""
    point_array = np.asarray(points, dtype=np.float32)
    probability_array = np.asarray(probabilities, dtype=np.float32)
    if point_array.ndim != 2 or point_array.shape[1] != 3:
        raise ValueError(f"Points must have shape (N, 3), got {point_array.shape}")
    if probability_array.shape != (point_array.shape[0],):
        raise ValueError(
            "Probabilities must have shape "
            f"({point_array.shape[0]},), got {probability_array.shape}"
        )
    if np.any(~np.isfinite(probability_array)) or np.any(
        (probability_array < 0.0) | (probability_array > 1.0)
    ):
        raise ValueError("Occupancy probabilities must be finite and in [0, 1]")

    cloud_dtype = np.dtype(
        [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("occupancy", "<f4")]
    )
    cloud_data = np.empty(point_array.shape[0], dtype=cloud_dtype)
    cloud_data["x"] = point_array[:, 0]
    cloud_data["y"] = point_array[:, 1]
    cloud_data["z"] = point_array[:, 2]
    cloud_data["occupancy"] = probability_array

    message = PointCloud2()
    message.header = header
    message.height = 1
    message.width = int(point_array.shape[0])
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(
            name="occupancy", offset=12, datatype=PointField.FLOAT32, count=1
        ),
    ]
    message.is_bigendian = False
    message.point_step = cloud_dtype.itemsize
    message.row_step = message.point_step * message.width
    message.is_dense = bool(np.all(np.isfinite(point_array)))
    message.data = cloud_data.tobytes()
    return message


def build_xyz_semantic_cloud(
    points: np.ndarray,
    labels: np.ndarray,
    confidences: np.ndarray,
    observations: np.ndarray,
    colors_rgb: np.ndarray,
    header: Header,
) -> PointCloud2:
    """Build confirmed semantic voxel centers with planner-readable fields."""
    point_array = np.asarray(points, dtype=np.float32)
    label_array = np.asarray(labels, dtype=np.uint8)
    confidence_array = np.asarray(confidences, dtype=np.float32)
    observation_array = np.asarray(observations, dtype=np.uint32)
    color_array = np.asarray(colors_rgb, dtype=np.uint8)
    count = point_array.shape[0]
    if point_array.ndim != 2 or point_array.shape[1] != 3:
        raise ValueError(f"Points must have shape (N, 3), got {point_array.shape}")
    if label_array.shape != (count,):
        raise ValueError("labels must have shape (N,)")
    if confidence_array.shape != (count,) or np.any(~np.isfinite(confidence_array)):
        raise ValueError("confidences must contain N finite values")
    if np.any((confidence_array < 0.0) | (confidence_array > 1.0)):
        raise ValueError("confidences must be in [0, 1]")
    if observation_array.shape != (count,):
        raise ValueError("observations must have shape (N,)")
    if color_array.shape != (count, 3):
        raise ValueError("colors_rgb must have shape (N, 3)")

    packed_rgb = (
        color_array[:, 0].astype(np.uint32) << 16
        | color_array[:, 1].astype(np.uint32) << 8
        | color_array[:, 2].astype(np.uint32)
    )
    cloud_dtype = np.dtype(
        {
            "names": [
                "x",
                "y",
                "z",
                "rgb",
                "semantic_label",
                "confidence",
                "observations",
            ],
            "formats": ["<f4", "<f4", "<f4", "<u4", "u1", "<f4", "<u4"],
            "offsets": [0, 4, 8, 12, 16, 20, 24],
            "itemsize": 28,
        }
    )
    cloud_data = np.zeros(count, dtype=cloud_dtype)
    cloud_data["x"] = point_array[:, 0]
    cloud_data["y"] = point_array[:, 1]
    cloud_data["z"] = point_array[:, 2]
    cloud_data["rgb"] = packed_rgb
    cloud_data["semantic_label"] = label_array
    cloud_data["confidence"] = confidence_array
    cloud_data["observations"] = observation_array

    message = PointCloud2()
    message.header = header
    message.height = 1
    message.width = count
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        PointField(
            name="semantic_label", offset=16, datatype=PointField.UINT8, count=1
        ),
        PointField(
            name="confidence", offset=20, datatype=PointField.FLOAT32, count=1
        ),
        PointField(
            name="observations", offset=24, datatype=PointField.UINT32, count=1
        ),
    ]
    message.is_bigendian = False
    message.point_step = cloud_dtype.itemsize
    message.row_step = message.point_step * count
    message.is_dense = bool(np.all(np.isfinite(point_array)))
    message.data = cloud_data.tobytes()
    return message


def read_xyz_cloud(message: PointCloud2) -> np.ndarray:
    """Decode XYZ float32 fields from an organized or unorganized cloud."""
    fields = {field.name: field for field in message.fields}
    missing = {name for name in ("x", "y", "z") if name not in fields}
    if missing:
        raise ValueError(f"Point cloud is missing XYZ fields: {sorted(missing)}")
    for name in ("x", "y", "z"):
        field = fields[name]
        if field.datatype != PointField.FLOAT32 or field.count != 1:
            raise ValueError(f"Point cloud field {name!r} must be one FLOAT32")
        if field.offset < 0 or field.offset + 4 > message.point_step:
            raise ValueError(f"Point cloud field {name!r} has an invalid offset")
    if message.point_step <= 0:
        raise ValueError("Point cloud point_step must be positive")
    width = int(message.width)
    height = int(message.height)
    if width < 0 or height < 0:
        raise ValueError("Point cloud dimensions must be non-negative")
    if width == 0 or height == 0:
        return np.empty((0, 3), dtype=np.float32)
    minimum_row_step = width * int(message.point_step)
    if message.row_step < minimum_row_step:
        raise ValueError("Point cloud row_step is smaller than packed row size")
    expected_size = int(message.row_step) * height
    if len(message.data) < expected_size:
        raise ValueError("Point cloud data is shorter than row_step * height")

    endian = ">f4" if message.is_bigendian else "<f4"
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [endian, endian, endian],
            "offsets": [fields[name].offset for name in ("x", "y", "z")],
            "itemsize": int(message.point_step),
        }
    )
    buffer = memoryview(message.data)
    if int(message.row_step) == minimum_row_step:
        cloud = np.frombuffer(buffer, dtype=dtype, count=width * height)
    else:
        rows = [
            np.frombuffer(
                buffer[row * int(message.row_step):], dtype=dtype, count=width
            )
            for row in range(height)
        ]
        cloud = np.concatenate(rows)
    return np.column_stack((cloud["x"], cloud["y"], cloud["z"])).astype(
        np.float32, copy=False
    )


def read_xyzuv_cloud(message: PointCloud2) -> tuple[np.ndarray, np.ndarray]:
    """Decode map-frame XYZ and aligned RGB pixel coordinates from a cloud."""
    points = read_xyz_cloud(message)
    fields = {field.name: field for field in message.fields}
    missing = {name for name in ("u", "v") if name not in fields}
    if missing:
        raise ValueError(f"Point cloud is missing pixel fields: {sorted(missing)}")
    for name in ("u", "v"):
        field = fields[name]
        if field.datatype != PointField.UINT32 or field.count != 1:
            raise ValueError(f"Point cloud field {name!r} must be one UINT32")
        if field.offset < 0 or field.offset + 4 > message.point_step:
            raise ValueError(f"Point cloud field {name!r} has an invalid offset")
    width = int(message.width)
    height = int(message.height)
    if width == 0 or height == 0:
        return points, np.empty((0, 2), dtype=np.int32)
    minimum_row_step = width * int(message.point_step)
    expected_size = int(message.row_step) * height
    if message.row_step < minimum_row_step or len(message.data) < expected_size:
        raise ValueError("Point cloud data layout is invalid")

    endian = ">u4" if message.is_bigendian else "<u4"
    dtype = np.dtype(
        {
            "names": ["u", "v"],
            "formats": [endian, endian],
            "offsets": [fields[name].offset for name in ("u", "v")],
            "itemsize": int(message.point_step),
        }
    )
    buffer = memoryview(message.data)
    if int(message.row_step) == minimum_row_step:
        cloud = np.frombuffer(buffer, dtype=dtype, count=width * height)
    else:
        rows = [
            np.frombuffer(
                buffer[row * int(message.row_step):], dtype=dtype, count=width
            )
            for row in range(height)
        ]
        cloud = np.concatenate(rows)
    pixels = np.column_stack((cloud["u"], cloud["v"]))
    if np.any(pixels > np.iinfo(np.int32).max):
        raise ValueError("Point cloud pixel fields exceed int32 range")
    return points, pixels.astype(np.int32, copy=False)
