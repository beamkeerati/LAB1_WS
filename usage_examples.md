# MPC Path Tracking with Dual Path Support

This system now supports two path sources:
1. **YAML file** (original functionality)
2. **Built-in switch_back course** (new functionality)

## Usage Examples

### 1. Using YAML Path (Default)

```bash
ros2 launch limo_controller mpc_single_evaluation.launch.py \
    path_type:=yaml \
    yaml_path_file:=path.yaml \
    experiment_name:=mpc_yaml_test \
    duration:=120.0 \
    target_speed:=1.1
```

### 2. Using Built-in Switch Back Course

```bash
ros2 launch limo_controller mpc_single_evaluation.launch.py \
    path_type:=switch_back \
    experiment_name:=mpc_switch_back_test \
    duration:=120.0 \
    target_speed:=0.8
```

### 3. Using Custom YAML File

```bash
ros2 launch limo_controller mpc_single_evaluation.launch.py \
    path_type:=yaml \
    yaml_path_file:=custom_path.yaml \
    experiment_name:=mpc_custom_path_test \
    duration:=120.0 \
    target_speed:=1.0
```

### 4. With Custom MPC Parameters

```bash
ros2 launch limo_controller mpc_single_evaluation.launch.py \
    path_type:=switch_back \
    experiment_name:=mpc_tuned_test \
    duration:=120.0 \
    target_speed:=1.2 \
    position_weight:=15.0 \
    yaw_weight:=20.0 \
    control_weight:=0.05 \
    horizon_length:=15 \
    control_dt:=0.08 \
    max_steer_deg:=12.0
```

## Parameter Description

### Path Configuration
- `path_type`: **"yaml"** or **"switch_back"**
- `yaml_path_file`: Name of YAML file in the path directory (only used when path_type=yaml)

### MPC Parameters
- `target_speed`: Target speed in m/s
- `horizon_length`: MPC prediction horizon length
- `control_dt`: MPC control time step
- `position_weight`: Position tracking weight in cost function
- `yaw_weight`: Yaw tracking weight in cost function  
- `control_weight`: Control effort weight in cost function
- `max_steer_deg`: Maximum steering angle in degrees

### Experiment Parameters
- `experiment_name`: Name for the experiment (used in output files)
- `duration`: Experiment duration in seconds
- `save_directory`: Directory to save evaluation results

## Path Types

### YAML Path
- Loads waypoints from a YAML file in `limo_controller/path/` directory
- YAML format:
```yaml
- x: 0.0
  y: 0.0
  yaw: 0.0
- x: 5.0
  y: 2.0
  yaw: 0.5
# ... more waypoints
```

### Switch Back Course
- Uses the built-in `get_switch_back_course()` function
- Generates a complex path with forward and reverse sections
- Includes sharp turns and direction changes
- Good for testing MPC performance on challenging maneuvers

## File Structure

```
limo_controller/
├── launch/
│   └── mpc_single_evaluation.launch.py    # Modified launch file
├── limo_controller/
│   ├── mpc.py                            # Modified MPC controller
│   ├── mpc_lib.py                        # MPC library functions
│   ├── mpc_evaluator.py                  # Performance evaluator
│   ├── PathPlanning/
│   │   └── CubicSpline/
│   │       └── cubic_spline_planner.py   # Spline path generation
│   └── utils/
│       └── angle.py                      # Angle utilities
└── path/
    ├── path.yaml                         # Default path file
    └── custom_path.yaml                  # Custom path files
```

## Output Files

The evaluator generates:
- `raw_data_*.json`: Raw sensor data
- `metrics_*.json`: Processed performance metrics
- `summary_stats_*.csv`: Summary statistics
- `comprehensive_analysis_*.png`: Performance plots
- `evaluation_report_*.md`: Detailed analysis report

## Switching Between Paths

You can easily switch between path types by changing the `path_type` parameter:

- For YAML paths: `path_type:=yaml`
- For switch back course: `path_type:=switch_back`

The system will automatically:
1. Load the appropriate path
2. Generate speed profiles
3. Initialize the MPC controller
4. Start path tracking

## Tips for Best Performance

1. **For YAML paths**: Ensure smooth waypoint transitions and appropriate yaw angles
2. **For switch back course**: Use lower target speeds (0.5-1.0 m/s) for better tracking
3. **Tuning**: Increase position_weight and yaw_weight for tighter tracking
4. **Control smoothness**: Increase control_weight to reduce aggressive control actions