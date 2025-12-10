# GLIM Configuration Files

This directory contains configuration files for different sensor setups.

## Available Configurations

- `avia.env` - Configuration for Livox AVIA scanner (default for MandEye datasets)

## Configuration Variables

| Variable | Description | Example |
|----------|-------------|---------|
| `INPUT_TOPIC` | ROS topic for input point cloud | `/livox/pointcloud` |
| `OUTPUT_DIR` | Output directory name for results | `results-glim` |
| `RECORD_TOPICS` | Topics to record from GLIM | `/glim_ros/aligned_points_corrected /glim_ros/odom_corrected` |

## Usage

Configs are automatically loaded by `run_benchmark.sh`:

```bash
./run_benchmark.sh avia /path/to/data/
```
