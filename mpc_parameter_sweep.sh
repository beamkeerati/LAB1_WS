#!/bin/bash

# MPC Parameter Sweep Automation Script
# This script automates running parameter sweep experiments for MPC controller evaluation

set -e  # Exit on any error

# Script configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${SCRIPT_DIR}/src/limo_controller/config/mpc_sweep_config.yaml"
RESULTS_BASE_DIR="${HOME}/mpc_sweep_results"
LOG_DIR="${RESULTS_BASE_DIR}/logs"
TIMEOUT_DURATION=125  # 5 seconds buffer over evaluation time
LAUNCH_PACKAGE="limo_controller"
LAUNCH_FILE="mpc_single_evaluation.launch.py"

# Validation function for workspace
validate_workspace() {
    if [[ ! -f "${SCRIPT_DIR}/src/limo_controller/package.xml" ]]; then
        log_error "This doesn't appear to be a valid ROS2 workspace with limo_controller package"
        log_error "Please run this script from your workspace root directory (e.g., ~/FRA532_MobileRobot/LAB1_WS/)"
        log_error "Expected structure:"
        log_error "  workspace_root/"
        log_error "  ├── src/limo_controller/"
        log_error "  ├── build/"
        log_error "  ├── install/"
        log_error "  └── mpc_parameter_sweep.sh"
        return 1
    fi
    
    if [[ ! -f "${SCRIPT_DIR}/install/setup.bash" ]]; then
        log_error "Workspace not built. Please build your workspace first:"
        log_error "  colcon build"
        return 1
    fi
    
    if [[ ! -f "$CONFIG_FILE" ]]; then
        log_error "Configuration file not found: $CONFIG_FILE"
        log_error "Please ensure mpc_sweep_config.yaml exists in src/limo_controller/config/"
        return 1
    fi
    
    # Check if workspace is sourced
    if [[ -z "$AMENT_PREFIX_PATH" ]] || [[ "$AMENT_PREFIX_PATH" != *"$SCRIPT_DIR/install"* ]]; then
        log_warning "Workspace doesn't appear to be sourced. Attempting to source it..."
        if [[ -f "${SCRIPT_DIR}/install/setup.bash" ]]; then
            source "${SCRIPT_DIR}/install/setup.bash"
            log_info "Workspace sourced successfully"
        else
            log_error "Failed to source workspace. Please run:"
            log_error "  source install/setup.bash"
            return 1
        fi
    fi
    
    return 0
}

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Global variables
CURRENT_EXPERIMENT=0
TOTAL_EXPERIMENTS=0
LAUNCH_PID=""
EXPERIMENT_COMPLETED=false
START_TIME=""

# Logging functions
log_info() {
    echo -e "${CYAN}[INFO]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

# Function to parse YAML configuration
parse_yaml_config() {
    if [[ ! -f "$CONFIG_FILE" ]]; then
        log_error "Configuration file not found: $CONFIG_FILE"
        exit 1
    fi
    
    log_info "Parsing parameter configuration from $CONFIG_FILE"
    
    # More robust YAML parsing with better error handling
    TARGET_SPEEDS=($(awk '/target_speed:/,/description:/' "$CONFIG_FILE" | grep -E '- [0-9]' | sed 's/.*- *\([0-9.]*\).*/\1/' | tr -d '[],' || echo ""))
    HORIZON_LENGTHS=($(awk '/horizon_length:/,/description:/' "$CONFIG_FILE" | grep -E '- [0-9]' | sed 's/.*- *\([0-9]*\).*/\1/' | tr -d '[],' || echo ""))
    CONTROL_DTS=($(awk '/control_dt:/,/description:/' "$CONFIG_FILE" | grep -E '- [0-9]' | sed 's/.*- *\([0-9.]*\).*/\1/' | tr -d '[],' || echo ""))
    POSITION_WEIGHTS=($(awk '/position_weight:/,/description:/' "$CONFIG_FILE" | grep -E '- [0-9]' | sed 's/.*- *\([0-9.]*\).*/\1/' | tr -d '[],' || echo ""))
    YAW_WEIGHTS=($(awk '/yaw_weight:/,/description:/' "$CONFIG_FILE" | grep -E '- [0-9]' | sed 's/.*- *\([0-9.]*\).*/\1/' | tr -d '[],' || echo ""))
    CONTROL_WEIGHTS=($(awk '/control_weight:/,/description:/' "$CONFIG_FILE" | grep -E '- [0-9]' | sed 's/.*- *\([0-9.]*\).*/\1/' | tr -d '[],' || echo ""))
    MAX_STEER_DEGS=($(awk '/max_steer_deg:/,/description:/' "$CONFIG_FILE" | grep -E '- [0-9]' | sed 's/.*- *\([0-9.]*\).*/\1/' | tr -d '[],' || echo ""))
    PATH_TYPES=($(awk '/path_type:/,/description:/' "$CONFIG_FILE" | grep -E '- "' | sed 's/.*- *"\([^"]*\)".*/\1/' || echo ""))
    
    # Debug: Show what we parsed
    log_info "DEBUG: Parsing results:"
    log_info "  TARGET_SPEEDS raw: $(awk '/target_speed:/,/description:/' "$CONFIG_FILE" | grep -E '- [0-9]')"
    log_info "  PATH_TYPES raw: $(awk '/path_type:/,/description:/' "$CONFIG_FILE" | grep -E '- "')"
    
    # Fallback to default values if parsing failed
    if [[ ${#TARGET_SPEEDS[@]} -eq 0 ]]; then
        log_warning "Failed to parse target_speed, using defaults"
        TARGET_SPEEDS=(0.5 0.8 1.1)
    fi
    
    if [[ ${#HORIZON_LENGTHS[@]} -eq 0 ]]; then
        log_warning "Failed to parse horizon_length, using defaults"
        HORIZON_LENGTHS=(8 12 16)
    fi
    
    if [[ ${#CONTROL_DTS[@]} -eq 0 ]]; then
        log_warning "Failed to parse control_dt, using defaults"
        CONTROL_DTS=(0.05 0.1)
    fi
    
    if [[ ${#POSITION_WEIGHTS[@]} -eq 0 ]]; then
        log_warning "Failed to parse position_weight, using defaults"
        POSITION_WEIGHTS=(5.0 15.0 40.0)
    fi
    
    if [[ ${#YAW_WEIGHTS[@]} -eq 0 ]]; then
        log_warning "Failed to parse yaw_weight, using defaults"
        YAW_WEIGHTS=(5.0 20.0 50.0)
    fi
    
    if [[ ${#CONTROL_WEIGHTS[@]} -eq 0 ]]; then
        log_warning "Failed to parse control_weight, using defaults"
        CONTROL_WEIGHTS=(0.1 0.5)
    fi
    
    if [[ ${#MAX_STEER_DEGS[@]} -eq 0 ]]; then
        log_warning "Failed to parse max_steer_deg, using defaults"
        MAX_STEER_DEGS=(10.0 15.0)
    fi
    
    if [[ ${#PATH_TYPES[@]} -eq 0 ]]; then
        log_warning "Failed to parse path_type, using defaults"
        PATH_TYPES=("yaml" "switch_back")
    fi
    
    # Calculate total number of experiments
    TOTAL_EXPERIMENTS=$((${#TARGET_SPEEDS[@]} * ${#HORIZON_LENGTHS[@]} * ${#CONTROL_DTS[@]} * ${#POSITION_WEIGHTS[@]} * ${#YAW_WEIGHTS[@]} * ${#CONTROL_WEIGHTS[@]} * ${#MAX_STEER_DEGS[@]} * ${#PATH_TYPES[@]}))
    
    log_info "Parameter ranges detected:"
    log_info "  Target speeds: ${TARGET_SPEEDS[*]}"
    log_info "  Horizon lengths: ${HORIZON_LENGTHS[*]}"
    log_info "  Control DTs: ${CONTROL_DTS[*]}"
    log_info "  Position weights: ${POSITION_WEIGHTS[*]}"
    log_info "  Yaw weights: ${YAW_WEIGHTS[*]}"
    log_info "  Control weights: ${CONTROL_WEIGHTS[*]}"
    log_info "  Max steer degrees: ${MAX_STEER_DEGS[*]}"
    log_info "  Path types: ${PATH_TYPES[*]}"
    log_info "Total experiments to run: $TOTAL_EXPERIMENTS"
}

# Function to setup directories
setup_directories() {
    log_info "Setting up result directories"
    mkdir -p "$RESULTS_BASE_DIR"
    mkdir -p "$LOG_DIR"
    
    # Create timestamp for this sweep session
    SWEEP_TIMESTAMP=$(date '+%Y%m%d_%H%M%S')
    SWEEP_DIR="${RESULTS_BASE_DIR}/sweep_${SWEEP_TIMESTAMP}"
    mkdir -p "$SWEEP_DIR"
    
    log_info "Results will be saved to: $SWEEP_DIR"
    echo "$SWEEP_DIR" > "${RESULTS_BASE_DIR}/.current_sweep_dir"
}

# Function to generate experiment name
generate_experiment_name() {
    local target_speed=$1
    local horizon_length=$2
    local control_dt=$3
    local position_weight=$4
    local yaw_weight=$5
    local control_weight=$6
    local max_steer_deg=$7
    local path_type=$8
    
    echo "mpc_ts${target_speed}_hl${horizon_length}_dt${control_dt}_pw${position_weight}_yw${yaw_weight}_cw${control_weight}_ms${max_steer_deg}_pt${path_type}"
}

# Function to wait for ROS2 to be ready
wait_for_ros() {
    log_info "Waiting for ROS2 to be ready..."
    local max_attempts=30
    local attempt=0
    
    while [[ $attempt -lt $max_attempts ]]; do
        if ros2 node list >/dev/null 2>&1; then
            log_success "ROS2 is ready"
            return 0
        fi
        sleep 1
        ((attempt++))
    done
    
    log_error "ROS2 failed to become ready after $max_attempts seconds"
    return 1
}


# Function to kill launch process and cleanup - FIXED
kill_launch_process() {
    if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
        log_info "Terminating launch process (PID: $LAUNCH_PID)"
        
        # Send SIGINT (Ctrl+C) first
        kill -INT "$LAUNCH_PID" 2>/dev/null || true
        
        # Wait for graceful shutdown - INCREASED timeout
        local timeout=30  # Increased from 10 to 30 seconds
        while [[ $timeout -gt 0 ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; do
            sleep 1
            ((timeout--))
        done
        
        # Force kill if still running
        if kill -0 "$LAUNCH_PID" 2>/dev/null; then
            log_warning "Force killing launch process"
            kill -KILL "$LAUNCH_PID" 2>/dev/null || true
        fi
        
        wait "$LAUNCH_PID" 2>/dev/null || true
        LAUNCH_PID=""
    fi
    
    # Additional cleanup - kill any remaining nodes
    log_info "Cleaning up remaining ROS2 nodes"
    pkill -f "mpc.py" 2>/dev/null || true
    pkill -f "mpc_evaluator.py" 2>/dev/null || true
    pkill -f "gazebo" 2>/dev/null || true
    
    # Wait for cleanup
    sleep 5  # Increased from 2 to 5 seconds
}

# Function to run single experiment - FIXED
run_experiment() {
    local target_speed=$1
    local horizon_length=$2
    local control_dt=$3
    local position_weight=$4
    local yaw_weight=$5
    local control_weight=$6
    local max_steer_deg=$7
    local path_type=$8
    
    # Generate experiment name with better formatting
    local experiment_name=$(generate_experiment_name "$target_speed" "$horizon_length" "$control_dt" "$position_weight" "$yaw_weight" "$control_weight" "$max_steer_deg" "$path_type")
    
    # Progress indication
    log_info "===================================================="
    log_info "Running experiment $((CURRENT_EXPERIMENT + 1))/$TOTAL_EXPERIMENTS"
    log_info "Experiment: $experiment_name"
    log_info "Parameters:"
    log_info "  target_speed: $target_speed m/s"
    log_info "  horizon_length: $horizon_length"
    log_info "  control_dt: $control_dt s"
    log_info "  position_weight: $position_weight"
    log_info "  yaw_weight: $yaw_weight"
    log_info "  control_weight: $control_weight"  
    log_info "  max_steer_deg: $max_steer_deg°"
    log_info "  path_type: $path_type"
    log_info "===================================================="
    
    # Create experiment-specific directory with better organization
    local exp_dir="${SWEEP_DIR}/${experiment_name}"
    local exp_results_dir="${exp_dir}/results"
    local exp_logs_dir="${exp_dir}/logs"
    
    mkdir -p "$exp_dir"
    mkdir -p "$exp_results_dir" 
    mkdir -p "$exp_logs_dir"
    
    # Create experiment metadata file
    cat > "${exp_dir}/experiment_info.yaml" << EOF
experiment_name: $experiment_name
experiment_number: $((CURRENT_EXPERIMENT + 1))
total_experiments: $TOTAL_EXPERIMENTS
timestamp: $(date '+%Y-%m-%d %H:%M:%S')
parameters:
  target_speed: $target_speed
  horizon_length: $horizon_length
  control_dt: $control_dt
  position_weight: $position_weight
  yaw_weight: $yaw_weight
  control_weight: $control_weight
  max_steer_deg: $max_steer_deg
  path_type: $path_type
status: running
EOF
    
    # Ensure clean state
    kill_launch_process
    
    # Clean up any previous evaluation results
    rm -rf /tmp/mpc_evaluation 2>/dev/null || true
    
    # Wait for system to be ready
    wait_for_ros
    sleep 5  # Increased from 3 to 5
    
    # Start the launch file with parameters, directing output to specific results directory
    local launch_cmd="ros2 launch $LAUNCH_PACKAGE $LAUNCH_FILE"
    launch_cmd+=" experiment_name:=$experiment_name"
    launch_cmd+=" duration:=$TIMEOUT_DURATION"
    launch_cmd+=" target_speed:=$target_speed"
    launch_cmd+=" horizon_length:=$horizon_length"
    launch_cmd+=" control_dt:=$control_dt"
    launch_cmd+=" save_directory:=$exp_results_dir"
    launch_cmd+=" position_weight:=$position_weight"
    launch_cmd+=" yaw_weight:=$yaw_weight"
    launch_cmd+=" control_weight:=$control_weight"
    launch_cmd+=" max_steer_deg:=$max_steer_deg"
    launch_cmd+=" path_type:=$path_type"
    
    # Add YAML path file parameter only if using yaml path type
    if [[ "$path_type" == "yaml" ]]; then
        launch_cmd+=" yaml_path_file:=path.yaml"
    fi
    
    log_info "Starting launch command: $launch_cmd"
    
    # Start the launch file in background and capture PID
    $launch_cmd > "${exp_logs_dir}/launch.log" 2>&1 &
    LAUNCH_PID=$!
    
    log_info "Launch process started with PID: $LAUNCH_PID"
    
    # Wait a moment for launch to initialize
    sleep 10  # Increased from 5 to 10
    
    # Check if launch process started successfully
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        log_error "Launch process failed to start or crashed immediately"
        log_error "Check log file: ${exp_logs_dir}/launch.log"
        
        # Update experiment status
        sed -i 's/status: running/status: failed_startup/' "${exp_dir}/experiment_info.yaml"
        echo "error: Launch process failed to start" >> "${exp_dir}/experiment_info.yaml"
        
        return 1
    fi
    
    # Monitor the experiment
    local experiment_start_time=$(date +%s)
    monitor_experiment "$experiment_name" "$TIMEOUT_DURATION" "$exp_dir"
    local experiment_end_time=$(date +%s)
    local experiment_duration=$((experiment_end_time - experiment_start_time))
    
    # Stop the launch process
    kill_launch_process
    
    # Wait for system cleanup and file writing
    sleep 10  # Increased from 3 to 10 seconds to allow evaluator to save files
    
    # Copy results from default location if they exist
    if [[ -d "/tmp/mpc_evaluation" ]]; then
        log_info "Copying results from /tmp/mpc_evaluation to $exp_results_dir"
        cp -r /tmp/mpc_evaluation/* "$exp_results_dir/" 2>/dev/null || true
        rm -rf /tmp/mpc_evaluation 2>/dev/null || true
    fi
    
    # FIXED: Check if we have any results - proper wildcard handling
    local has_results=false
    if ls "$exp_results_dir"/summary_stats_*.json 1> /dev/null 2>&1 || ls "$exp_results_dir"/raw_data_*.json 1> /dev/null 2>&1; then
        has_results=true
    fi
    
    # Also check for any CSV files
    if ls "$exp_results_dir"/*.csv 1> /dev/null 2>&1; then
        has_results=true
    fi
    
    # Update experiment metadata with results
    local final_status="completed"
    if [[ "$has_results" == "false" ]]; then
        final_status="no_results"
        
        # Log the contents of the results directory for debugging
        log_warning "No results found in $exp_results_dir"
        log_warning "Directory contents:"
        ls -la "$exp_results_dir" 2>&1 | while read line; do
            log_warning "  $line"
        done
        
        # Check the launch log for errors
        if [[ -f "${exp_logs_dir}/launch.log" ]]; then
            log_warning "Last 20 lines of launch log:"
            tail -20 "${exp_logs_dir}/launch.log" | while read line; do
                log_warning "  $line"
            done
        fi
    fi
    
    cat >> "${exp_dir}/experiment_info.yaml" << EOF
duration_seconds: $experiment_duration
end_timestamp: $(date '+%Y-%m-%d %H:%M:%S')
status: $final_status
has_results: $has_results
results_directory: $exp_results_dir
logs_directory: $exp_logs_dir
EOF
    
    # Create a summary of results if available
    if [[ "$has_results" == "true" ]]; then
        create_experiment_summary "$exp_dir"
        log_success "Experiment $experiment_name completed with results"
    else
        log_warning "Experiment $experiment_name completed but no results found"
    fi
    
    return 0
}

# Modified monitor experiment function with better completion detection
monitor_experiment() {
    local experiment_name=$1
    local timeout=$2
    local exp_dir=$3
    local start_time=$(date +%s)
    
    log_info "Monitoring experiment: $experiment_name (timeout: ${timeout}s)"
    
    EXPERIMENT_COMPLETED=false
    local last_position_x=0.0
    local last_position_y=0.0
    local stuck_count=0
    local max_stuck_count=20  # 20 seconds of being stuck
    
    while true; do
        current_time=$(date +%s)
        elapsed=$((current_time - start_time))
        
        # Check if launch process is still running
        if [[ -n "$LAUNCH_PID" ]] && ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
            log_warning "Launch process terminated unexpectedly"
            EXPERIMENT_COMPLETED=true
            break
        fi
        
        # Check for goal reached or experiment failed by monitoring topics
        if timeout 5 ros2 topic echo /mpc/experiment_status --once 2>/dev/null | grep -q "goal_reached\|experiment_failed"; then
            log_success "Experiment completed (goal reached or failed)"
            EXPERIMENT_COMPLETED=true
            # Give evaluator time to save results
            sleep 5
            break
        fi
        
        # Check if evaluator has already saved results (new check)
        if ls "${exp_dir}/results"/summary_stats_*.json 1> /dev/null 2>&1; then
            log_success "Results already saved by evaluator"
            EXPERIMENT_COMPLETED=true
            sleep 2
            break
        fi
        
        # Monitor robot position to detect if stuck
        if command -v ros2 &> /dev/null; then
            robot_pose=$(timeout 3 ros2 topic echo /odometry/ground_truth geometry_msgs/msg/Pose --once 2>/dev/null | grep -A 3 "position:" | grep -E "x:|y:" | awk '{print $2}' | tr '\n' ' ')
            if [[ -n "$robot_pose" ]]; then
                read -r current_x current_y <<< "$robot_pose"
                
                # Check if robot moved significantly (threshold: 0.1m)
                x_diff=$(echo "$current_x - $last_position_x" | bc -l 2>/dev/null || echo "0")
                y_diff=$(echo "$current_y - $last_position_y" | bc -l 2>/dev/null || echo "0")
                movement=$(echo "sqrt($x_diff*$x_diff + $y_diff*$y_diff)" | bc -l 2>/dev/null || echo "0")
                
                if (( $(echo "$movement < 0.1" | bc -l 2>/dev/null || echo "1") )); then
                    ((stuck_count++))
                    if [[ $stuck_count -ge $max_stuck_count ]]; then
                        log_warning "Robot appears stuck (no movement for ${max_stuck_count}s), ending experiment"
                        EXPERIMENT_COMPLETED=true
                        break
                    fi
                else
                    stuck_count=0
                    last_position_x=$current_x
                    last_position_y=$current_y
                fi
            fi
        fi
        
        # Check timeout - but give extra time for evaluator to finish
        if [[ $elapsed -ge $((timeout + 10)) ]]; then
            log_warning "Experiment timed out after ${elapsed} seconds (including grace period)"
            EXPERIMENT_COMPLETED=true
            break
        elif [[ $elapsed -ge $timeout ]]; then
            log_info "Evaluation duration reached, waiting for evaluator to save results..."
        fi
        
        # Progress indicator every 10 seconds
        if [[ $((elapsed % 10)) -eq 0 ]] && [[ $elapsed -gt 0 ]]; then
            remaining=$((timeout - elapsed))
            if [[ $remaining -lt 0 ]]; then
                log_info "In grace period... waiting for evaluator to complete"
            elif [[ -n "$robot_pose" ]]; then
                log_info "Experiment running... ${elapsed}s elapsed, ${remaining}s remaining (pos: ${current_x:-0}, ${current_y:-0}, stuck: ${stuck_count}s)"
            else
                log_info "Experiment running... ${elapsed}s elapsed, ${remaining}s remaining"
            fi
        fi
        
        sleep 1
    done
    
    log_info "Experiment monitoring completed after ${elapsed} seconds"
}

# Function to create experiment summary
create_experiment_summary() {
    local exp_dir=$1
    local summary_file="${exp_dir}/EXPERIMENT_SUMMARY.md"
    
    log_info "Creating experiment summary: $summary_file"
    
    # Extract experiment info
    local exp_name=$(grep "experiment_name:" "${exp_dir}/experiment_info.yaml" | cut -d' ' -f2-)
    local exp_num=$(grep "experiment_number:" "${exp_dir}/experiment_info.yaml" | cut -d' ' -f2)
    local duration=$(grep "duration_seconds:" "${exp_dir}/experiment_info.yaml" | cut -d' ' -f2)
    local status=$(grep "status:" "${exp_dir}/experiment_info.yaml" | tail -1 | cut -d' ' -f2)
    
    cat > "$summary_file" << EOF
# Experiment Summary: $exp_name

## Basic Information
- **Experiment Number:** $exp_num / $TOTAL_EXPERIMENTS
- **Duration:** ${duration}s
- **Status:** $status
- **Timestamp:** $(grep "timestamp:" "${exp_dir}/experiment_info.yaml" | cut -d' ' -f2-)

## Parameters
EOF
    
    # Add parameters
    grep -A 10 "parameters:" "${exp_dir}/experiment_info.yaml" | grep "  " >> "$summary_file"
    
    cat >> "$summary_file" << EOF

## Files Generated
### Results Directory: \`results/\`
EOF
    
    # List result files
    if [[ -d "${exp_dir}/results" ]]; then
        find "${exp_dir}/results" -type f -name "*.json" -o -name "*.csv" -o -name "*.png" -o -name "*.pdf" -o -name "*.md" | sort | while read -r file; do
            echo "- \`$(basename "$file")\`" >> "$summary_file"
        done
    fi
    
    cat >> "$summary_file" << EOF

### Logs Directory: \`logs/\`
- \`launch.log\` - Launch file output and errors

## Quick Access
- **Main Results:** \`results/summary_stats_*.csv\`
- **Performance Plots:** \`results/comprehensive_analysis_*.png\`
- **Detailed Report:** \`results/evaluation_report_*.md\`
- **Launch Logs:** \`logs/launch.log\`

---
*Generated automatically by MPC Parameter Sweep Script*
EOF
    
    log_success "Experiment summary created: $summary_file"
}

# Function to create sweep summary
create_sweep_summary() {
    local summary_file="${SWEEP_DIR}/SWEEP_SUMMARY.md"
    local csv_summary="${SWEEP_DIR}/experiments_summary.csv"
    
    log_info "Creating comprehensive sweep summary"
    
    # Create markdown summary
    cat > "$summary_file" << EOF
# MPC Parameter Sweep Results Summary

## Sweep Overview
- **Sweep ID:** sweep_${SWEEP_TIMESTAMP}
- **Start Time:** $START_TIME
- **End Time:** $(date '+%Y-%m-%d %H:%M:%S')
- **Total Experiments:** $TOTAL_EXPERIMENTS
- **Results Directory:** $SWEEP_DIR

## Parameter Space Explored
- **Target Speeds:** ${TARGET_SPEEDS[*]} m/s
- **Horizon Lengths:** ${HORIZON_LENGTHS[*]}
- **Control DTs:** ${CONTROL_DTS[*]} s
- **Position Weights:** ${POSITION_WEIGHTS[*]}
- **Yaw Weights:** ${YAW_WEIGHTS[*]}
- **Control Weights:** ${CONTROL_WEIGHTS[*]}
- **Max Steer Degrees:** ${MAX_STEER_DEGS[*]}°
- **Path Types:** ${PATH_TYPES[*]}

## Experiment Results Overview

| Experiment | Status | Duration | Target Speed | Horizon | Path Type | Has Results |
|------------|--------|----------|--------------|---------|-----------|-------------|
EOF
    
    # Create CSV header
    echo "experiment_name,experiment_number,status,duration_seconds,target_speed,horizon_length,control_dt,position_weight,yaw_weight,control_weight,max_steer_deg,path_type,has_results" > "$csv_summary"
    
    # Process each experiment directory
    local completed_count=0
    local failed_count=0
    local with_results_count=0
    
    for exp_dir in "${SWEEP_DIR}"/mpc_ts*; do
        if [[ -d "$exp_dir" && -f "$exp_dir/experiment_info.yaml" ]]; then
            local exp_name=$(basename "$exp_dir")
            local exp_num=$(grep "experiment_number:" "$exp_dir/experiment_info.yaml" | cut -d' ' -f2 || echo "N/A")
            local status=$(grep "status:" "$exp_dir/experiment_info.yaml" | tail -1 | cut -d' ' -f2 || echo "unknown")
            local duration=$(grep "duration_seconds:" "$exp_dir/experiment_info.yaml" | cut -d' ' -f2 || echo "N/A")
            local has_results=$(grep "has_results:" "$exp_dir/experiment_info.yaml" | cut -d' ' -f2 || echo "false")
            
            # Extract parameters
            local target_speed=$(grep "target_speed:" "$exp_dir/experiment_info.yaml" | cut -d' ' -f2 || echo "N/A")
            local horizon_length=$(grep "horizon_length:" "$exp_dir/experiment_info.yaml" | cut -d' ' -f2 || echo "N/A")
            local control_dt=$(grep "control_dt:" "$exp_dir/experiment_info.yaml" | cut -d' ' -f2 || echo "N/A")
            local position_weight=$(grep "position_weight:" "$exp_dir/experiment_info.yaml" | cut -d' ' -f2 || echo "N/A")
            local yaw_weight=$(grep "yaw_weight:" "$exp_dir/experiment_info.yaml" | cut -d' ' -f2 || echo "N/A")
            local control_weight=$(grep "control_weight:" "$exp_dir/experiment_info.yaml" | cut -d' ' -f2 || echo "N/A")
            local max_steer_deg=$(grep "max_steer_deg:" "$exp_dir/experiment_info.yaml" | cut -d' ' -f2 || echo "N/A")
            local path_type=$(grep "path_type:" "$exp_dir/experiment_info.yaml" | cut -d' ' -f2 || echo "N/A")
            
            # Count statistics
            case "$status" in
                "completed") ((completed_count++)) ;;
                "failed"*|"no_results") ((failed_count++)) ;;
            esac
            
            if [[ "$has_results" == "true" ]]; then
                ((with_results_count++))
            fi
            
            # Add to markdown table
            echo "| $exp_name | $status | ${duration}s | $target_speed | $horizon_length | $path_type | $has_results |" >> "$summary_file"
            
            # Add to CSV
            echo "$exp_name,$exp_num,$status,$duration,$target_speed,$horizon_length,$control_dt,$position_weight,$yaw_weight,$control_weight,$max_steer_deg,$path_type,$has_results" >> "$csv_summary"
        fi
    done
    
    # Add statistics to markdown
    cat >> "$summary_file" << EOF

## Statistics
- **Completed Successfully:** $completed_count experiments
- **Failed or No Results:** $failed_count experiments  
- **With Valid Results:** $with_results_count experiments
- **Success Rate:** $(( (with_results_count * 100) / TOTAL_EXPERIMENTS ))%

## Directory Structure
\`\`\`
sweep_${SWEEP_TIMESTAMP}/
├── SWEEP_SUMMARY.md          # This summary file
├── experiments_summary.csv   # CSV data for analysis
├── sweep_summary.txt         # Legacy text summary
└── experiments/
    ├── mpc_ts0.5_hl8_dt0.05_.../ 
    │   ├── EXPERIMENT_SUMMARY.md
    │   ├── experiment_info.yaml
    │   ├── results/
    │   │   ├── summary_stats_*.csv
    │   │   ├── comprehensive_analysis_*.png
    │   │   ├── evaluation_report_*.md
    │   │   └── raw_data_*.json
    │   └── logs/
    │       └── launch.log
    └── ...
\`\`\`

## Quick Analysis Commands

### View all experiment statuses:
\`\`\`bash
grep -r "status:" */experiment_info.yaml | grep -v "running"
\`\`\`

### Find experiments with results:
\`\`\`bash
find . -name "summary_stats_*.csv" | wc -l
\`\`\`

### View best performing experiments:
\`\`\`bash
# Look for experiments with lowest position errors (manual inspection needed)
find . -name "summary_stats_*.csv" -exec head -1 {} \; -exec grep "position_errors" {} \;
\`\`\`

## Configuration Used
- **Config File:** $CONFIG_FILE
- **Timeout Duration:** ${TIMEOUT_DURATION}s per experiment
- **Launch Package:** $LAUNCH_PACKAGE
- **Launch File:** $LAUNCH_FILE

---
*Generated automatically by MPC Parameter Sweep Script v2.0*
*For detailed analysis, examine individual experiment directories*
EOF

    # Also create the legacy text summary for backward compatibility
    create_legacy_summary
    
    log_success "Comprehensive sweep summary created:"
    log_info "  - Markdown summary: $summary_file"
    log_info "  - CSV data: $csv_summary" 
    log_info "  - Legacy summary: ${SWEEP_DIR}/sweep_summary.txt"
}

# Legacy summary function for backward compatibility
create_legacy_summary() {
    local summary_file="${SWEEP_DIR}/sweep_summary.txt"
    
    cat > "$summary_file" << EOF
MPC Parameter Sweep Summary
===========================

Sweep ID: sweep_${SWEEP_TIMESTAMP}
Start Time: $START_TIME
End Time: $(date '+%Y-%m-%d %H:%M:%S')
Total Experiments: $TOTAL_EXPERIMENTS
Results Directory: $SWEEP_DIR

Parameter Ranges:
- Target Speeds: ${TARGET_SPEEDS[*]}
- Horizon Lengths: ${HORIZON_LENGTHS[*]}
- Control DTs: ${CONTROL_DTS[*]}
- Position Weights: ${POSITION_WEIGHTS[*]}
- Yaw Weights: ${YAW_WEIGHTS[*]}
- Control Weights: ${CONTROL_WEIGHTS[*]}
- Max Steer Degrees: ${MAX_STEER_DEGS[*]}
- Path Types: ${PATH_TYPES[*]}

Configuration File: $CONFIG_FILE

Results Structure:
Each experiment directory contains:
- EXPERIMENT_SUMMARY.md - Quick overview and file list
- experiment_info.yaml - Detailed metadata
- results/ - Performance metrics, plots, reports
- logs/ - Launch file logs and debug info

Use the comprehensive analysis tools to process and compare results across experiments.
For quick overview, see SWEEP_SUMMARY.md and experiments_summary.csv
EOF
}

# Cleanup function for script termination
cleanup() {
    log_info "Cleaning up..."
    kill_launch_process
    
    if [[ -n "$START_TIME" ]]; then
        create_sweep_summary
    fi
    
    log_info "Parameter sweep terminated"
    exit 0
}

# Signal handlers
trap cleanup SIGINT SIGTERM

# Main function
main() {
    log_info "Starting MPC Parameter Sweep"
    START_TIME=$(date '+%Y-%m-%d %H:%M:%S')
    
    # Validate workspace and environment
    if ! validate_workspace; then
        exit 1
    fi
    
    # Check dependencies
    if ! command -v ros2 &> /dev/null; then
        log_error "ROS2 not found. Please source your ROS2 installation:"
        log_error "  source /opt/ros/humble/setup.bash"
        log_error "  source install/setup.bash"
        exit 1
    fi
    
    if ! command -v bc &> /dev/null; then
        log_error "bc (calculator) not found. Please install it:"
        log_error "  sudo apt update && sudo apt install bc"
        exit 1
    fi
    
    log_info "✅ Workspace validation passed"
    log_info "📁 Working from: $SCRIPT_DIR"
    
    # Parse configuration
    parse_yaml_config
    
    # Setup directories
    setup_directories
    
    # Wait for initial ROS2 readiness
    wait_for_ros
    
    log_info "Starting parameter sweep with $TOTAL_EXPERIMENTS experiments"
    log_info "Estimated total time: $(( (TIMEOUT_DURATION + 10) * TOTAL_EXPERIMENTS / 60 )) minutes"
    echo ""
    
    # Main sweep loop
    CURRENT_EXPERIMENT=0
    local failed_experiments=0
    local successful_experiments=0
    
    for target_speed in "${TARGET_SPEEDS[@]}"; do
        for horizon_length in "${HORIZON_LENGTHS[@]}"; do
            for control_dt in "${CONTROL_DTS[@]}"; do
                for position_weight in "${POSITION_WEIGHTS[@]}"; do
                    for yaw_weight in "${YAW_WEIGHTS[@]}"; do
                        for control_weight in "${CONTROL_WEIGHTS[@]}"; do
                            for max_steer_deg in "${MAX_STEER_DEGS[@]}"; do
                                for path_type in "${PATH_TYPES[@]}"; do
                                    # Run the experiment
                                    if run_experiment "$target_speed" "$horizon_length" "$control_dt" "$position_weight" "$yaw_weight" "$control_weight" "$max_steer_deg" "$path_type"; then
                                        ((successful_experiments++))
                                        log_success "✅ Experiment $((CURRENT_EXPERIMENT + 1))/$TOTAL_EXPERIMENTS completed successfully"
                                    else
                                        ((failed_experiments++))
                                        log_error "❌ Experiment $((CURRENT_EXPERIMENT + 1))/$TOTAL_EXPERIMENTS failed"
                                    fi
                                    
                                    ((CURRENT_EXPERIMENT++))
                                    
                                    # Progress summary every 10 experiments
                                    if [[ $((CURRENT_EXPERIMENT % 10)) -eq 0 ]]; then
                                        local remaining_experiments=$((TOTAL_EXPERIMENTS - CURRENT_EXPERIMENT))
                                        local estimated_remaining_time=$(( (TIMEOUT_DURATION + 10) * remaining_experiments / 60 ))
                                        log_info "📊 Progress: $CURRENT_EXPERIMENT/$TOTAL_EXPERIMENTS (${successful_experiments} successful, ${failed_experiments} failed)"
                                        log_info "⏱️  Estimated remaining time: ${estimated_remaining_time} minutes"
                                        echo ""
                                    fi
                                    
                                    # Brief pause between experiments
                                    sleep 2
                                done
                            done
                        done
                    done
                done
            done
        done
    done
    
    echo ""
    log_success "🎉 All experiments completed!"
    log_info "📈 Final Statistics:"
    log_info "  Total Experiments: $TOTAL_EXPERIMENTS"
    log_info "  Successful: $successful_experiments"
    log_info "  Failed: $failed_experiments"
    if [[ $TOTAL_EXPERIMENTS -gt 0 ]]; then
        log_info "  Success Rate: $(( (successful_experiments * 100) / TOTAL_EXPERIMENTS ))%"
    else
        log_info "  Success Rate: N/A (no experiments run)"
    fi
    
    create_sweep_summary
    
    echo ""
    log_info "🔍 Quick Analysis:"
    log_info "  View results: cd $SWEEP_DIR"
    log_info "  Run analysis: ./mpc_analysis.sh $SWEEP_DIR"
    log_info "  Best performers: ./mpc_analysis.sh -b -n 5 $SWEEP_DIR"
    
    log_success "Parameter sweep completed successfully!"
    log_info "Results directory: $SWEEP_DIR"
}

# Script usage
usage() {
    cat << EOF
Usage: $0 [OPTIONS]

MPC Parameter Sweep Automation Script

This script automatically runs parameter sweep experiments for the MPC controller
by iterating through all parameter combinations defined in mpc_sweep_config.yaml.

OPTIONS:
    -h, --help          Show this help message
    -c, --config FILE   Use custom configuration file (default: ./src/limo_controller/config/mpc_sweep_config.yaml)
    -t, --timeout SEC   Set experiment timeout in seconds (default: 125)
    -d, --dir DIR       Set results base directory (default: ~/mpc_sweep_results)

EXAMPLES:
    $0                                  # Run with default settings
    $0 -c custom_config.yaml           # Use custom config file
    $0 -t 180 -d /tmp/results          # Custom timeout and results directory

REQUIREMENTS:
    - ROS2 must be sourced and available
    - limo_controller package must be built and available
    - Gazebo simulation environment should be ready

The script will:
1. Parse the parameter configuration file
2. Create result directories
3. For each parameter combination:
   - Start the launch file with specific parameters
   - Monitor for completion (goal reached or timeout)  
   - Terminate the launch file (Ctrl+C equivalent)
   - Wait for cleanup before starting next experiment
4. Generate a summary of all experiments

Results are organized by experiment name and include:
- Performance metrics and plots
- Raw data logs
- Evaluation reports
EOF
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            usage
            exit 0
            ;;
        -c|--config)
            CONFIG_FILE="$2"
            shift 2
            ;;
        -t|--timeout)
            TIMEOUT_DURATION="$2"
            shift 2
            ;;
        -d|--dir)
            RESULTS_BASE_DIR="$2"
            shift 2
            ;;
        *)
            log_error "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

# Run main function
main "$@"