"""Launch geometry, occupancy, and optional 2D semantics in the map frame."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from pathlib import Path


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("semantic_mapping"))
    default_model_dir = Path.home() / ".cache/tinynav/semantic_models/segformer_b0_ade20k"
    default_classes = str(share / "config" / "semantic_classes.yaml")
    default_mapping = str(share / "config" / "ade20k_navigation_mapping.yaml")
    common = PythonLaunchDescriptionSource(
        str(share / "launch" / "semantic_mapping.launch.py")
    )
    target_frame = LaunchConfiguration("target_frame")
    publish_once = LaunchConfiguration("publish_once")
    enable_occupancy = LaunchConfiguration("enable_occupancy")
    output_directory = LaunchConfiguration("output_directory")
    input_directory = LaunchConfiguration("input_directory")
    allow_input_frame_override = LaunchConfiguration("allow_input_frame_override")
    enable_semantic_perception = LaunchConfiguration(
        "enable_semantic_perception"
    )
    enable_semantic_fusion = LaunchConfiguration("enable_semantic_fusion")
    semantic_backend = LaunchConfiguration("semantic_backend")
    precomputed_mask_directory = LaunchConfiguration(
        "precomputed_mask_directory"
    )
    precomputed_manifest = LaunchConfiguration("precomputed_manifest")
    semantic_classes_file = LaunchConfiguration("semantic_classes_file")
    semantic_engine = LaunchConfiguration("semantic_engine")
    semantic_model_config = LaunchConfiguration("semantic_model_config")
    semantic_preprocessor_config = LaunchConfiguration(
        "semantic_preprocessor_config"
    )
    semantic_label_mapping = LaunchConfiguration("semantic_label_mapping")
    semantic_min_confidence = LaunchConfiguration("semantic_min_confidence")
    return LaunchDescription(
        [
            DeclareLaunchArgument("target_frame", default_value="map"),
            DeclareLaunchArgument("publish_once", default_value="false"),
            DeclareLaunchArgument("enable_occupancy", default_value="true"),
            DeclareLaunchArgument("output_directory", default_value=""),
            DeclareLaunchArgument("input_directory", default_value=""),
            DeclareLaunchArgument(
                "allow_input_frame_override", default_value="false"
            ),
            DeclareLaunchArgument(
                "enable_semantic_perception", default_value="false"
            ),
            DeclareLaunchArgument("enable_semantic_fusion", default_value="false"),
            DeclareLaunchArgument("semantic_backend", default_value="precomputed"),
            DeclareLaunchArgument("precomputed_mask_directory", default_value=""),
            DeclareLaunchArgument(
                "precomputed_manifest", default_value="manifest.yaml"
            ),
            DeclareLaunchArgument(
                "semantic_classes_file", default_value=default_classes
            ),
            DeclareLaunchArgument(
                "semantic_engine", default_value=str(default_model_dir / "model_fp16.engine")
            ),
            DeclareLaunchArgument(
                "semantic_model_config", default_value=str(default_model_dir / "config.json")
            ),
            DeclareLaunchArgument(
                "semantic_preprocessor_config",
                default_value=str(default_model_dir / "preprocessor_config.json"),
            ),
            DeclareLaunchArgument(
                "semantic_label_mapping", default_value=default_mapping
            ),
            DeclareLaunchArgument("semantic_min_confidence", default_value="0.35"),
            IncludeLaunchDescription(
                common,
                launch_arguments={
                    "target_frame": target_frame,
                    "use_sim_time": "false",
                    "publish_once": publish_once,
                    "enable_occupancy": enable_occupancy,
                    "output_directory": output_directory,
                    "input_directory": input_directory,
                    "allow_input_frame_override": allow_input_frame_override,
                    "enable_semantic_perception": enable_semantic_perception,
                    "enable_semantic_fusion": enable_semantic_fusion,
                    "semantic_backend": semantic_backend,
                    "precomputed_mask_directory": precomputed_mask_directory,
                    "precomputed_manifest": precomputed_manifest,
                    "semantic_classes_file": semantic_classes_file,
                    "semantic_engine": semantic_engine,
                    "semantic_model_config": semantic_model_config,
                    "semantic_preprocessor_config": semantic_preprocessor_config,
                    "semantic_label_mapping": semantic_label_mapping,
                    "semantic_min_confidence": semantic_min_confidence,
                }.items(),
            ),
        ]
    )
