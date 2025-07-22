# ROS2 Streaming for MobilityGen

This document explains how to use the ROS2 streaming functionality in MobilityGen, which allows you to stream sensor data via ROS2 topics and save it as ROS2 bag files while recording.

## Overview

The ROS2 streaming functionality provides several components:

1. **ROS2Writer**: A ROS2 node that publishes sensor data to ROS2 topics
2. **ROS2BagWriter**: A ROS2 node that streams data via topics AND saves to ROS2 bag files
3. **HybridWriter**: A unified interface that can write to files, stream via ROS2, or both
4. **Modified replay scripts**: Updated scripts that support ROS2 streaming
5. **Enhanced extension**: Updated MobilityGen extension with ROS2 controls

## Features

- **Real-time streaming**: Stream sensor data in real-time via ROS2 topics
- **ROS2 bag recording**: Save data to ROS2 bag files (.db3) for later analysis
- **Multiple data types**: RGB images, depth maps, segmentation, normals, and occupancy maps
- **Compressed images**: Optional compressed image topics for bandwidth efficiency
- **Hybrid mode**: Write to files and stream via ROS2 simultaneously
- **Flexible configuration**: Choose between file-only, ROS2-only, or hybrid modes
- **Integrated UI**: ROS2 controls directly in the MobilityGen extension

## Prerequisites

1. **ROS2 Installation**: Make sure ROS2 is installed and sourced
   ```bash
   # Source ROS2 (adjust path as needed)
   source /opt/ros/humble/setup.bash
   ```

2. **Python Dependencies**: Install required Python packages
   ```bash
   pip install rclpy cv-bridge opencv-python
   ```

3. **Isaac Sim**: Ensure Isaac Sim is properly installed and configured

## Usage

### 1. Using ROS2 Streaming in the Extension

The easiest way to use ROS2 streaming is directly in the MobilityGen extension:

1. **Launch Isaac Sim** with MobilityGen extension
2. **Build a scenario** (robot + environment)
3. **Enable ROS2** in the extension UI:
   - Check "Enable ROS2" checkbox
   - Set namespace (default: `/mobility_gen`)
   - Optionally enable compression
4. **Click "Start Recording"** - this will now:
   - Save data to files (as before)
   - Stream data via ROS2 topics
   - Save data to a ROS2 bag file (.db3)

### 2. Basic ROS2 Streaming Example

Run the example script to test ROS2 streaming:

```bash
# ROS2-only streaming
python examples/ros2_streaming_example.py --mode ros2 --steps 100

# Hybrid mode (file + ROS2)
python examples/ros2_streaming_example.py --mode hybrid --file-path ./output --steps 100

# With compression enabled
python examples/ros2_streaming_example.py --mode ros2 --compression --steps 100
```

### 2. Replay with ROS2 Streaming

Use the ROS2-enabled replay script to stream recorded data:

```bash
# Stream recorded data via ROS2
python scripts/replay_implementation_ros2.py \
    --input_path /path/to/recording \
    --writer_mode ros2 \
    --ros2_namespace /mobility_gen

# Hybrid mode - save to files and stream via ROS2
python scripts/replay_implementation_ros2.py \
    --input_path /path/to/recording \
    --output_path ./replay_output \
    --writer_mode hybrid \
    --ros2_namespace /mobility_gen
```

### 3. Working with ROS2 Bag Files

After recording with ROS2 enabled, you'll have a `.db3` bag file. You can analyze and extract data from it:

```bash
# Analyze a bag file
python examples/ros2_bag_example.py --bag-file ~/MobilityGenData/recordings/2025-01-17T16:44:33.006521.db3 --analyze

# Extract RGB images from a bag file
python examples/ros2_bag_example.py --bag-file ~/MobilityGenData/recordings/2025-01-17T16:44:33.006521.db3 --extract-images --output-dir ./extracted_images

# Plot robot trajectory
python examples/ros2_bag_example.py --bag-file ~/MobilityGenData/recordings/2025-01-17T16:44:33.006521.db3 --plot-trajectory
```

### 4. Programmatic Usage

```python
import rclpy
from omni.ext.mobility_gen.ros2_writer import ROS2Writer
from omni.ext.mobility_gen.hybrid_writer import HybridWriter, WriterMode

# Initialize ROS2
rclpy.init()

# Create ROS2 writer
ros2_writer = ROS2Writer(
    node_name="my_ros2_writer",
    namespace="/mobility_gen",
    enable_compression=True
)

# Create hybrid writer
writer = HybridWriter.create_hybrid_writer(
    file_path="./output",
    ros2_writer=ros2_writer
)

# Use the writer (same interface as file-based Writer)
writer.write_state_dict_common(state_dict, step)
writer.write_state_dict_rgb(rgb_data, step)
writer.write_state_dict_segmentation(seg_data, step)
writer.write_state_dict_depth(depth_data, step)
writer.write_state_dict_normals(normals_data, step)

# Clean up
writer.destroy()
rclpy.shutdown()
```

## ROS2 Topics

The following topics are published when using ROS2 streaming:

### Image Topics
- `/mobility_gen/rgb/{camera_name}` - RGB images (sensor_msgs/Image)
- `/mobility_gen/rgb/{camera_name}/compressed` - Compressed RGB images (sensor_msgs/CompressedImage)
- `/mobility_gen/segmentation/{camera_name}` - Segmentation images (sensor_msgs/Image)
- `/mobility_gen/depth/{camera_name}` - Depth images (sensor_msgs/Image)
- `/mobility_gen/normals/{camera_name}` - Normal maps (sensor_msgs/Image)

### State Topics
- `/mobility_gen/common_state` - Common state data as JSON string (std_msgs/String)
- `/mobility_gen/robot_pose` - Robot pose (geometry_msgs/PoseStamped)
- `/mobility_gen/robot_twist` - Robot twist (geometry_msgs/TwistStamped)
- `/mobility_gen/occupancy_map` - Occupancy map (nav_msgs/OccupancyGrid)

## Viewing Streamed Data

### 1. List Available Topics
```bash
ros2 topic list
```

### 2. View RGB Images
```bash
# Using rqt_image_view
ros2 run rqt_image_view rqt_image_view

# Or using command line
ros2 run image_transport republish raw /mobility_gen/rgb/camera_left
```

### 3. View Occupancy Map
```bash
# Using RViz
ros2 run rviz2 rviz2

# Add OccupancyGrid display and set topic to /mobility_gen/occupancy_map
```

### 4. Monitor State Data
```bash
# View common state
ros2 topic echo /mobility_gen/common_state

# View robot pose
ros2 topic echo /mobility_gen/robot_pose

# View robot twist
ros2 topic echo /mobility_gen/robot_twist
```

## Configuration Options

### Writer Modes
- **FILE**: Write only to files (original behavior)
- **ROS2**: Stream only via ROS2 topics
- **HYBRID**: Write to files and stream via ROS2

### ROS2 Options
- `--ros2_namespace`: ROS2 namespace for topics (default: `/mobility_gen`)
- `--enable_compression`: Enable compressed image topics
- `--writer_mode`: Choose writer mode (file/ros2/hybrid)

### Example Configurations

```bash
# File-only mode (original behavior)
python scripts/replay_implementation_ros2.py \
    --input_path /path/to/recording \
    --output_path ./output \
    --writer_mode file

# ROS2-only mode
python scripts/replay_implementation_ros2.py \
    --input_path /path/to/recording \
    --writer_mode ros2 \
    --ros2_namespace /my_robot

# Hybrid mode with compression
python scripts/replay_implementation_ros2.py \
    --input_path /path/to/recording \
    --output_path ./output \
    --writer_mode hybrid \
    --enable_compression \
    --ros2_namespace /mobility_gen
```

## Integration with Existing Code

The ROS2 streaming functionality is designed to be a drop-in replacement for the existing file-based writer. You can easily modify existing code to use ROS2 streaming:

### Before (File-based)
```python
from omni.ext.mobility_gen.writer import Writer

writer = Writer("./output")
writer.write_state_dict_rgb(rgb_data, step)
```

### After (ROS2 streaming)
```python
from omni.ext.mobility_gen.hybrid_writer import HybridWriter, WriterMode
from omni.ext.mobility_gen.ros2_writer import ROS2Writer

# Initialize ROS2
rclpy.init()

# Create ROS2 writer
ros2_writer = ROS2Writer(namespace="/mobility_gen")

# Create hybrid writer
writer = HybridWriter.create_ros2_writer(ros2_writer)

# Use the same interface
writer.write_state_dict_rgb(rgb_data, step)

# Clean up
writer.destroy()
rclpy.shutdown()
```

## Troubleshooting

### Common Issues

1. **ROS2 not found**: Make sure ROS2 is installed and sourced
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Import errors**: Ensure all dependencies are installed
   ```bash
   pip install rclpy cv-bridge opencv-python
   ```

3. **Topic not visible**: Check if the ROS2 node is running
   ```bash
   ros2 node list
   ros2 topic list
   ```

4. **Image not displaying**: Verify image format and encoding
   ```bash
   ros2 topic echo /mobility_gen/rgb/camera_left --once
   ```

### Debug Mode

Enable debug logging for more detailed information:

```python
import rclpy
rclpy.init()
rclpy.logging.set_logger_level('mobility_gen_ros2_writer', rclpy.logging.LoggingSeverity.DEBUG)
```

## Performance Considerations

1. **Network bandwidth**: Compressed image topics can significantly reduce bandwidth usage
2. **CPU usage**: ROS2 streaming adds some CPU overhead compared to file writing
3. **Memory usage**: Images are kept in memory during publishing
4. **Latency**: Real-time streaming may have higher latency than file writing

## Future Enhancements

- **Custom message types**: Support for custom ROS2 message types
- **QoS configuration**: Configurable Quality of Service settings
- **Multi-camera support**: Better support for stereo and multi-camera setups
- **TF2 integration**: Automatic transform publishing
- **Parameter server**: Configuration via ROS2 parameter server

## Contributing

To contribute to the ROS2 streaming functionality:

1. Follow the existing code style and patterns
2. Add appropriate error handling and logging
3. Include unit tests for new functionality
4. Update this documentation for any new features 