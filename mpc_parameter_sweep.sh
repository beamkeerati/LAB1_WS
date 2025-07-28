#!/bin/bash

# Improved MPC Parameter Sweep Script
# Author: Generated for LIMO MPC Controller
# Version: 2.2 (Integer Arithmetic Fix)

set -e  # Exit on any error

# =============================================================================
# CONFIGURATION
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"  # Script is in the workspace root (LAB1_WS)
CONFIG_FILE="$WORKSPACE_DIR/src/limo_controller/config/mpc_sweep_config.yaml"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
RESULTS_BASE_DIR="$HOME/mpc_sweep_results/sweep_$TIMESTAMP"

# Colors and formatting
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color
BOLD='\033[1m'

# Timing configuration
DEFAULT_EXPERIMENT_DURATION=60
CLEANUP_DELAY=5
STARTUP_DELAY=8
MONITORING_INTERVAL=20
FORCE_KILL_DELAY=10

# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

log_info() {
    echo -e "${BLUE}[INFO]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1"
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

log_header() {
    echo -e "${BOLD}${PURPLE}$1${NC}"
}

# Improved cleanup function - PROVEN COMMANDS
cleanup_ros2() {
    log_info "🔥 Cleaning up ALL ROS2 and Gazebo processes (AGGRESSIVE MODE)..."
    
    # This kills ALL ROS2 and Gazebo processes system-wide - PROVEN TO WORK
    sudo pkill -f ros2
    sudo pkill -f ros  
    sudo pkill -f gazebo
    sudo pkill -f ign
    sudo pkill -f ignition
    sudo killall -9 ruby
    sudo killall -9 python3
    
    # Clean up ROS2 daemon
    ros2 daemon stop
    ros2 daemon start
    
    # Final cleanup - kill any remaining Python processes
    sudo pkill -9 -f ".py"
    
    sleep $CLEANUP_DELAY
}

# Check if ROS2 is ready
check_ros2_ready() {
    local max_attempts=10
    local attempt=1
    
    while [ $attempt -le $max_attempts ]; do
        if ros2 node list >/dev/null 2>&1; then
            return 0
        fi
        sleep 1
        ((attempt++))
    done
    return 1
}

# Parse YAML configuration
parse_config() {
    if ! command -v python3 >/dev/null 2>&1; then
        log_error "Python3 is required for YAML parsing"
        exit 1
    fi
    
    log_info "Parsing parameter configuration from $CONFIG_FILE"
    
    # Check if config file exists
    if [[ ! -f "$CONFIG_FILE" ]]; then
        log_error "Configuration file not found: $CONFIG_FILE"
        exit 1
    fi
    
    # Use Python to parse YAML and generate parameter combinations
    python3 << EOF
import yaml
import itertools
import sys

try:
    with open('$CONFIG_FILE', 'r') as f:
        config = yaml.safe_load(f)
    
    # Extract parameters
    params = config['parameters']
    
    # Print parameter ranges for logging
    for key, values in params.items():
        print(f"PARAM_{key.upper()}=(" + " ".join(map(str, values)) + ")")
    
    # Generate all combinations
    keys = list(params.keys())
    values = [params[key] for key in keys]
    combinations = list(itertools.product(*values))
    
    print(f"TOTAL_EXPERIMENTS={len(combinations)}")
    # FIX: Cast duration to an integer to prevent shell arithmetic errors
    duration = int(config.get('experiment', {}).get('duration', $DEFAULT_EXPERIMENT_DURATION))
    print(f"EXPERIMENT_DURATION={duration}")
    
    # Write combinations to temporary file
    with open('/tmp/mpc_combinations.txt', 'w') as f:
        for i, combo in enumerate(combinations):
            f.write(f"{i}:" + ":".join(map(str, combo)) + "\n")
    
except Exception as e:
    print(f"ERROR: Failed to parse config: {e}", file=sys.stderr)
    sys.exit(1)
EOF
    
    if [[ $? -ne 0 ]]; then
        log_error "Failed to parse configuration file"
        exit 1
    fi
    
    # Source the generated parameter ranges
    source <(python3 << EOF
import yaml
with open('$CONFIG_FILE', 'r') as f:
    config = yaml.safe_load(f)
params = config['parameters']
for key, values in params.items():
    print(f"PARAM_{key.upper()}=(" + " ".join(map(str, values)) + ")")
config_exp = config.get('experiment', {})
print(f"TOTAL_EXPERIMENTS=$(wc -l < /tmp/mpc_combinations.txt)")
# FIX: Cast duration to an integer to prevent shell arithmetic errors
duration = int(config_exp.get('duration', 60))
print(f"EXPERIMENT_DURATION={duration}")
EOF
)
}

# Generate experiment name
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

# Run single experiment
run_experiment() {
    local exp_num=$1
    local exp_name=$2
    local target_speed=$3
    local horizon_length=$4
    local control_dt=$5
    local position_weight=$6
    local yaw_weight=$7
    local control_weight=$8
    local max_steer_deg=$9
    local path_type=${10}
    
    local exp_dir="$RESULTS_BASE_DIR/$exp_name"
    local results_dir="$exp_dir/results"
    
    # Create experiment directory
    mkdir -p "$results_dir"
    
    log_header "===================================================="
    log_info "Running experiment $exp_num/$TOTAL_EXPERIMENTS"
    log_info "Experiment: $exp_name"
    log_info "Parameters:"
    log_info "  target_speed: $target_speed m/s"
    log_info "  horizon_length: $horizon_length"
    log_info "  control_dt: $control_dt s"
    log_info "  position_weight: $position_weight"
    log_info "  yaw_weight: $yaw_weight"
    log_info "  control_weight: $control_weight"
    log_info "  max_steer_deg: ${max_steer_deg}°"
    log_info "  path_type: $path_type"
    log_header "===================================================="
    
    # Cleanup before starting
    cleanup_ros2
    
    # Wait for ROS2 to be ready
    log_info "Waiting for ROS2 to be ready..."
    if check_ros2_ready; then
        log_success "ROS2 is ready"
    else
        log_error "ROS2 failed to start properly"
        return 1
    fi
    
    sleep $STARTUP_DELAY
    
    # Build launch command
    local launch_cmd="ros2 launch limo_controller mpc_single_evaluation.launch.py"
    launch_cmd+=" experiment_name:=$exp_name"
    launch_cmd+=" duration:=$EXPERIMENT_DURATION"
    launch_cmd+=" target_speed:=$target_speed"
    launch_cmd+=" horizon_length:=$horizon_length"
    launch_cmd+=" control_dt:=$control_dt"
    launch_cmd+=" save_directory:=$results_dir"
    launch_cmd+=" position_weight:=$position_weight"
    launch_cmd+=" yaw_weight:=$yaw_weight"
    launch_cmd+=" control_weight:=$control_weight"
    launch_cmd+=" max_steer_deg:=$max_steer_deg"
    launch_cmd+=" path_type:=$path_type"
    
    if [[ "$path_type" == "yaml" ]]; then
        launch_cmd+=" yaml_path_file:=path.yaml"
    fi
    
    log_info "Starting launch command: $launch_cmd"
    
    # Start experiment in background
    $launch_cmd > "$exp_dir/launch.log" 2>&1 &
    local launch_pid=$!
    
    log_info "Launch process started with PID: $launch_pid"
    
    # Monitor experiment
    local elapsed=0
    local timeout=$((EXPERIMENT_DURATION + 30))  # Add buffer
    
    sleep 10  # Let the system start up
    log_info "Monitoring experiment: $exp_name (timeout: ${timeout}s)"
    
    while [[ $elapsed -lt $timeout ]]; do
        if ! kill -0 $launch_pid 2>/dev/null; then
            log_info "Launch process completed naturally"
            break
        fi
        
        sleep $MONITORING_INTERVAL
        elapsed=$((elapsed + MONITORING_INTERVAL))
        
        if [[ $elapsed -lt $timeout ]]; then
            local remaining=$((timeout - elapsed))
            log_info "Experiment running... ${elapsed}s elapsed, ${remaining}s remaining"
        fi
    done
    
    # Check if experiment completed or needs to be terminated
    if [[ $elapsed -ge $EXPERIMENT_DURATION ]]; then
        log_info "Evaluation duration reached, waiting for evaluator to save results..."
        sleep 10  # Give evaluator time to save
        
        # Check if results were saved
        if ls "$results_dir"/*.json >/dev/null 2>&1; then
            log_success "Results already saved by evaluator"
        else
            log_warning "Results not found, extending wait time..."
            sleep 10
        fi
    fi
    
    log_info "Experiment monitoring completed after $elapsed seconds"
    
    # Terminate launch process if still running
    if kill -0 $launch_pid 2>/dev/null; then
        log_info "Terminating launch process (PID: $launch_pid)"
        kill -TERM $launch_pid 2>/dev/null || true
        sleep $FORCE_KILL_DELAY
        
        if kill -0 $launch_pid 2>/dev/null; then
            log_warning "Force killing launch process"
            kill -KILL $launch_pid 2>/dev/null || true
        fi
    fi
    
    # Final cleanup
    cleanup_ros2
    
    # Create experiment summary
    create_experiment_summary "$exp_dir" "$exp_name" "$target_speed" "$horizon_length" "$control_dt" "$position_weight" "$yaw_weight" "$control_weight" "$max_steer_deg" "$path_type"
    
    # Check if experiment was successful
    if ls "$results_dir"/*.json >/dev/null 2>&1; then
        log_success "Experiment $exp_name completed with results"
        return 0
    else
        log_error "Experiment $exp_name failed - no results found"
        return 1
    fi
}

# Create experiment summary
create_experiment_summary() {
    local exp_dir=$1
    local exp_name=$2
    local target_speed=$3
    local horizon_length=$4
    local control_dt=$5
    local position_weight=$6
    local yaw_weight=$7
    local control_weight=$8
    local max_steer_deg=$9
    local path_type=${10}
    
    local summary_file="$exp_dir/EXPERIMENT_SUMMARY.md"
    
    log_info "Creating experiment summary: $summary_file"
    
    cat > "$summary_file" << EOF
# MPC Experiment Summary

**Experiment Name:** $exp_name  
**Date:** $(date '+%Y-%m-%d %H:%M:%S')  
**Duration:** ${EXPERIMENT_DURATION}s  

## Parameters

| Parameter | Value |
|-----------|-------|
| Target Speed | $target_speed m/s |
| Horizon Length | $horizon_length |
| Control DT | $control_dt s |
| Position Weight | $position_weight |
| Yaw Weight | $yaw_weight |
| Control Weight | $control_weight |
| Max Steer Angle | ${max_steer_deg}° |
| Path Type | $path_type |

## Results Location

- **Results Directory:** \`$exp_dir/results/\`
- **Launch Log:** \`$exp_dir/launch.log\`
- **Summary:** \`$summary_file\`

## Files Generated

EOF
    
    if [[ -d "$exp_dir/results" ]]; then
        echo "### Result Files:" >> "$summary_file"
        find "$exp_dir/results" -type f -name "*.json" -o -name "*.csv" -o -name "*.png" -o -name "*.md" | sort | while read file; do
            echo "- \`$(basename "$file")\`" >> "$summary_file"
        done
    else
        echo "❌ **No results directory found**" >> "$summary_file"
    fi
    
    echo "" >> "$summary_file"
    echo "---" >> "$summary_file"
    echo "*Generated by MPC Parameter Sweep Script*" >> "$summary_file"
    
    log_success "Experiment summary created: $summary_file"
}

# =============================================================================
# MAIN EXECUTION
# =============================================================================

main() {
    log_header "Starting MPC Parameter Sweep"
    
    # Validate environment
    if [[ ! -d "$WORKSPACE_DIR" ]]; then
        log_error "Workspace directory not found: $WORKSPACE_DIR"
        exit 1
    fi
    
    if ! command -v ros2 >/dev/null 2>&1; then
        log_error "ROS2 not found. Please source your ROS2 installation."
        exit 1
    fi
    
    log_success "✅ Workspace validation passed"
    log_info "📁 Working from: $WORKSPACE_DIR"
    
    # Parse configuration
    parse_config
    
    # Log parameter ranges
    log_info "Parameter ranges detected:"
    echo "$(python3 << EOF
import yaml
with open('$CONFIG_FILE', 'r') as f:
    config = yaml.safe_load(f)
params = config['parameters']
for key, values in params.items():
    print(f"  {key.replace('_', ' ').title()}s: {' '.join(map(str, values))}")
EOF
)"
    
    log_info "Total experiments to run: $TOTAL_EXPERIMENTS"
    local estimated_time=$((TOTAL_EXPERIMENTS * (EXPERIMENT_DURATION + 45) / 60))
    log_info "Estimated total time: $estimated_time minutes"
    
    # Setup results directory
    log_info "Setting up result directories"
    mkdir -p "$RESULTS_BASE_DIR"
    log_info "Results will be saved to: $RESULTS_BASE_DIR"
    
    # Initial cleanup and setup
    cleanup_ros2
    
    # Check ROS2 readiness
    log_info "Waiting for ROS2 to be ready..."
    if check_ros2_ready; then
        log_success "ROS2 is ready"
    else
        log_error "ROS2 is not ready. Please check your installation."
        exit 1
    fi
    
    log_info "Starting parameter sweep with $TOTAL_EXPERIMENTS experiments"
    
    # Track experiment results
    local successful_experiments=0
    local failed_experiments=0
    
    # Read combinations and run experiments
    local exp_num=1
    while IFS=':' read -r index target_speed horizon_length control_dt position_weight yaw_weight control_weight max_steer_deg path_type; do
        local exp_name=$(generate_experiment_name "$target_speed" "$horizon_length" "$control_dt" "$position_weight" "$yaw_weight" "$control_weight" "$max_steer_deg" "$path_type")
        
        if run_experiment "$exp_num" "$exp_name" "$target_speed" "$horizon_length" "$control_dt" "$position_weight" "$yaw_weight" "$control_weight" "$max_steer_deg" "$path_type"; then
            ((successful_experiments++))
        else
            ((failed_experiments++))
        fi
        
        ((exp_num++))
        
        # Brief pause between experiments
        sleep 2
        
    done < /tmp/mpc_combinations.txt
    
    # Final summary
    log_header "=========================================="
    log_header "PARAMETER SWEEP COMPLETED"
    log_header "=========================================="
    log_success "✅ Successful experiments: $successful_experiments"
    log_error "❌ Failed experiments: $failed_experiments"
    log_info "📁 Results directory: $RESULTS_BASE_DIR"
    
    # Create overall summary
    create_overall_summary "$successful_experiments" "$failed_experiments"
    
    if [[ $successful_experiments -gt 0 ]]; then
        log_success "🎉 Parameter sweep completed with $successful_experiments successful experiments!"
    else
        log_error "💥 Parameter sweep completed with no successful experiments"
        exit 1
    fi
}

create_overall_summary() {
    local successful=$1
    local failed=$2
    local total=$((successful + failed))
    
    local overall_summary="$RESULTS_BASE_DIR/OVERALL_SUMMARY.md"
    
    cat > "$overall_summary" << EOF
# MPC Parameter Sweep - Overall Summary

**Date:** $(date '+%Y-%m-%d %H:%M:%S')  
**Total Experiments:** $total  
**Successful:** $successful  
**Failed:** $failed  
**Success Rate:** $(( successful * 100 / total ))%

## Results Directory Structure

\`\`\`
$RESULTS_BASE_DIR/
├── OVERALL_SUMMARY.md (this file)
EOF
    
    find "$RESULTS_BASE_DIR" -mindepth 1 -maxdepth 1 -type d | sort | while read dir; do
        local exp_name=$(basename "$dir")
        echo "├── $exp_name/" >> "$overall_summary"
        echo "│   ├── EXPERIMENT_SUMMARY.md" >> "$overall_summary"
        echo "│   ├── launch.log" >> "$overall_summary"
        echo "│   └── results/" >> "$overall_summary"
    done
    
    echo "\`\`\`" >> "$overall_summary"
    echo "" >> "$overall_summary"
    echo "## Analysis" >> "$overall_summary"
    echo "" >> "$overall_summary"
    echo "To analyze the results, check individual experiment directories for:" >> "$overall_summary"
    echo "- \`results/*.json\` - Raw data and metrics" >> "$overall_summary"
    echo "- \`results/*.csv\` - Summary statistics" >> "$overall_summary"
    echo "- \`results/*.png\` - Performance plots" >> "$overall_summary"
    echo "- \`results/*.md\` - Detailed analysis reports" >> "$overall_summary"
    echo "" >> "$overall_summary"
    echo "---" >> "$overall_summary"
    echo "*Generated by MPC Parameter Sweep Script v2.2*" >> "$overall_summary"
    
    log_success "Overall summary created: $overall_summary"
}

# Handle script interruption
trap 'log_warning "Script interrupted by user"; cleanup_ros2; exit 130' SIGINT SIGTERM

# Run main function
main "$@"