#!/bin/bash

# MPC Parameter Sweep Results Analysis Script
# This script helps analyze the results from the parameter sweep

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Default values
RESULTS_DIR=""
TOP_N=10
METRIC="position_errors"
SHOW_SUMMARY=false
SHOW_BEST=false
SHOW_WORST=false
SHOW_CORRELATIONS=false
GENERATE_PLOTS=false

# Logging functions
log_info() {
    echo -e "${CYAN}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to show usage
usage() {
    cat << EOF
Usage: $0 [OPTIONS] RESULTS_DIRECTORY

MPC Parameter Sweep Results Analysis Script

This script analyzes the results from MPC parameter sweep experiments.

ARGUMENTS:
    RESULTS_DIRECTORY   Path to sweep results directory (e.g., ~/mpc_sweep_results/sweep_20250128_143000)

OPTIONS:
    -m, --metric METRIC     Metric to analyze (default: position_errors)
                           Options: position_errors, heading_errors, cross_track_errors, 
                                   control_effort, control_smoothness, velocity_errors
    -n, --top N            Show top N best/worst results (default: 10)
    -s, --summary          Show quick summary only
    -b, --best             Show best performing experiments
    -w, --worst            Show worst performing experiments
    -c, --correlations     Show parameter correlations with performance
    -p, --plots            Generate comparison plots (requires Python with matplotlib)
    -h, --help             Show this help message

EXAMPLES:
    $0 ~/mpc_sweep_results/sweep_20250128_143000                    # Basic analysis
    $0 -m heading_errors -n 5 -b results/                          # Top 5 best heading performance
    $0 -s results/                                                  # Quick summary only
    $0 -c -p results/                                              # Full analysis with plots

METRICS DESCRIPTION:
    position_errors     - RMS position tracking error (lower is better)
    heading_errors      - RMS heading tracking error (lower is better)  
    cross_track_errors  - Cross-track deviation from path (lower is better)
    control_effort      - Average control magnitude (lower is smoother)
    control_smoothness  - Control rate changes (lower is smoother)
    velocity_errors     - Velocity tracking error (lower is better)
EOF
}

# Function to find latest sweep directory if not specified
find_latest_sweep() {
    local base_dir="${HOME}/mpc_sweep_results"
    if [[ -d "$base_dir" ]]; then
        local latest=$(find "$base_dir" -maxdepth 1 -type d -name "sweep_*" | sort | tail -1)
        if [[ -n "$latest" ]]; then
            echo "$latest"
            return 0
        fi
    fi
    return 1
}

# Function to validate results directory
validate_results_dir() {
    local dir=$1
    
    if [[ ! -d "$dir" ]]; then
        log_error "Results directory not found: $dir"
        return 1
    fi
    
    if [[ ! -f "$dir/experiments_summary.csv" ]]; then
        log_error "No experiments_summary.csv found. This doesn't appear to be a valid sweep results directory."
        return 1
    fi
    
    local exp_count=$(find "$dir" -maxdepth 1 -type d -name "mpc_ts*" | wc -l)
    if [[ $exp_count -eq 0 ]]; then
        log_error "No experiment directories found in $dir"
        return 1
    fi
    
    log_info "Found $exp_count experiment directories in $dir"
    return 0
}

# Function to show quick summary
show_summary() {
    local dir=$1
    
    log_info "=== MPC Parameter Sweep Summary ==="
    
    if [[ -f "$dir/SWEEP_SUMMARY.md" ]]; then
        # Extract key info from markdown summary
        local total_exp=$(grep "Total Experiments:" "$dir/SWEEP_SUMMARY.md" | cut -d' ' -f4 || echo "Unknown")
        local completed=$(grep "Completed Successfully:" "$dir/SWEEP_SUMMARY.md" | cut -d' ' -f4 || echo "Unknown")
        local with_results=$(grep "With Valid Results:" "$dir/SWEEP_SUMMARY.md" | cut -d' ' -f5 || echo "Unknown")
        local success_rate=$(grep "Success Rate:" "$dir/SWEEP_SUMMARY.md" | cut -d' ' -f4 || echo "Unknown")
        
        echo "📊 Total Experiments: $total_exp"
        echo "✅ Completed Successfully: $completed"
        echo "📈 With Valid Results: $with_results"
        echo "🎯 Success Rate: $success_rate"
    else
        # Fallback to CSV analysis
        local total_exp=$(tail -n +2 "$dir/experiments_summary.csv" | wc -l)
        local completed=$(tail -n +2 "$dir/experiments_summary.csv" | grep "completed" | wc -l)
        local with_results=$(tail -n +2 "$dir/experiments_summary.csv" | grep "true" | wc -l)
        local success_rate=$(( (with_results * 100) / total_exp ))
        
        echo "📊 Total Experiments: $total_exp"
        echo "✅ Completed Successfully: $completed"
        echo "📈 With Valid Results: $with_results"
        echo "🎯 Success Rate: ${success_rate}%"
    fi
    
    echo ""
    echo "📁 Results Directory: $dir"
    echo "📋 Summary Files:"
    [[ -f "$dir/SWEEP_SUMMARY.md" ]] && echo "   - SWEEP_SUMMARY.md (detailed overview)"
    [[ -f "$dir/experiments_summary.csv" ]] && echo "   - experiments_summary.csv (data for analysis)"
    echo ""
}

# Function to extract metric from summary stats JSON
extract_metric() {
    local json_file=$1
    local metric=$2
    local stat_type=${3:-"rms"}  # Default to RMS
    
    if [[ -f "$json_file" ]]; then
        # Use Python to parse JSON if available, otherwise use basic grep/sed
        if command -v python3 &> /dev/null; then
            python3 -c "
import json, sys
try:
    with open('$json_file', 'r') as f:
        data = json.load(f)
    value = data.get('$metric', {}).get('$stat_type', 'N/A')
    print(f'{value:.6f}' if isinstance(value, (int, float)) else 'N/A')
except:
    print('N/A')
"
        else
            # Fallback to basic text processing
            grep -A 10 "\"$metric\":" "$json_file" 2>/dev/null | grep "\"$stat_type\":" | head -1 | sed 's/.*: *\([0-9.]*\).*/\1/' || echo "N/A"
        fi
    else
        echo "N/A"
    fi
}

# Function to analyze experiments by metric
analyze_by_metric() {
    local dir=$1
    local metric=$2
    local top_n=$3
    local show_best=$4
    local show_worst=$5
    
    log_info "=== Analyzing by $metric ==="
    
    # Create temporary file for analysis
    local temp_file=$(mktemp)
    local analysis_file=$(mktemp)
    
    # Header
    echo "experiment_name,metric_value,target_speed,horizon_length,control_dt,position_weight,yaw_weight,control_weight,max_steer_deg,path_type" > "$temp_file"
    
    # Process each experiment
    for exp_dir in "$dir"/mpc_ts*; do
        if [[ -d "$exp_dir" && -f "$exp_dir/experiment_info.yaml" ]]; then
            local exp_name=$(basename "$exp_dir")
            
            # Find summary stats file
            local stats_file=$(find "$exp_dir/results" -name "summary_stats_*.json" 2>/dev/null | head -1)
            local metric_value=$(extract_metric "$stats_file" "$metric" "rms")
            
            if [[ "$metric_value" != "N/A" ]]; then
                # Extract parameters
                local target_speed=$(grep "target_speed:" "$exp_dir/experiment_info.yaml" | cut -d' ' -f2)
                local horizon_length=$(grep "horizon_length:" "$exp_dir/experiment_info.yaml" | cut -d' ' -f2)
                local control_dt=$(grep "control_dt:" "$exp_dir/experiment_info.yaml" | cut -d' ' -f2)
                local position_weight=$(grep "position_weight:" "$exp_dir/experiment_info.yaml" | cut -d' ' -f2)
                local yaw_weight=$(grep "yaw_weight:" "$exp_dir/experiment_info.yaml" | cut -d' ' -f2)
                local control_weight=$(grep "control_weight:" "$exp_dir/experiment_info.yaml" | cut -d' ' -f2)
                local max_steer_deg=$(grep "max_steer_deg:" "$exp_dir/experiment_info.yaml" | cut -d' ' -f2)
                local path_type=$(grep "path_type:" "$exp_dir/experiment_info.yaml" | cut -d' ' -f2)
                
                echo "$exp_name,$metric_value,$target_speed,$horizon_length,$control_dt,$position_weight,$yaw_weight,$control_weight,$max_steer_deg,$path_type" >> "$temp_file"
            fi
        fi
    done
    
    # Sort by metric value (ascending - lower is better for most metrics)
    tail -n +2 "$temp_file" | sort -t',' -k2 -n > "$analysis_file"
    
    local total_valid=$(wc -l < "$analysis_file")
    log_info "Found $total_valid experiments with valid $metric data"
    
    if [[ $total_valid -eq 0 ]]; then
        log_warning "No valid data found for metric: $metric"
        rm "$temp_file" "$analysis_file"
        return 1
    fi
    
    # Show best performers
    if [[ "$show_best" == "true" ]] || [[ "$show_best" == "" && "$show_worst" == "" ]]; then
        echo ""
        echo "🏆 TOP $top_n BEST PERFORMING (lowest $metric):"
        echo "Rank | Experiment | $metric | Target Speed | Horizon | Path | Position Weight | Yaw Weight"
        echo "----|------------|---------|--------------|---------|------|----------------|------------"
        
        head -n "$top_n" "$analysis_file" | nl -nln | while IFS=',' read -r rank exp_name metric_value target_speed horizon_length control_dt position_weight yaw_weight control_weight max_steer_deg path_type; do
            printf "%4s | %-25s | %8.4f | %12s | %7s | %8s | %15s | %10s\n" \
                "$rank" "$exp_name" "$metric_value" "$target_speed" "$horizon_length" "$path_type" "$position_weight" "$yaw_weight"
        done
    fi
    
    # Show worst performers
    if [[ "$show_worst" == "true" ]]; then
        echo ""
        echo "⚠️  TOP $top_n WORST PERFORMING (highest $metric):"
        echo "Rank | Experiment | $metric | Target Speed | Horizon | Path | Position Weight | Yaw Weight"
        echo "----|------------|---------|--------------|---------|------|----------------|------------"
        
        tail -n "$top_n" "$analysis_file" | tac | nl -nln | while IFS=',' read -r rank exp_name metric_value target_speed horizon_length control_dt position_weight yaw_weight control_weight max_steer_deg path_type; do
            printf "%4s | %-25s | %8.4f | %12s | %7s | %8s | %15s | %10s\n" \
                "$rank" "$exp_name" "$metric_value" "$target_speed" "$horizon_length" "$path_type" "$position_weight" "$yaw_weight"
        done
    fi
    
    # Basic statistics
    if command -v python3 &> /dev/null; then
        echo ""
        echo "📈 STATISTICS FOR $metric:"
        python3 -c "
import sys
import statistics

values = []
with open('$analysis_file', 'r') as f:
    for line in f:
        try:
            value = float(line.split(',')[1])
            values.append(value)
        except:
            continue

if values:
    print(f'   Mean: {statistics.mean(values):.6f}')
    print(f'   Median: {statistics.median(values):.6f}')
    print(f'   Std Dev: {statistics.stdev(values):.6f}' if len(values) > 1 else '   Std Dev: N/A')
    print(f'   Min: {min(values):.6f}')
    print(f'   Max: {max(values):.6f}')
    print(f'   Range: {max(values) - min(values):.6f}')
"
    fi
    
    # Cleanup
    rm "$temp_file" "$analysis_file"
}

# Function to show parameter correlations
show_correlations() {
    local dir=$1
    local metric=$2
    
    log_info "=== Parameter Correlations with $metric ==="
    
    if ! command -v python3 &> /dev/null; then
        log_warning "Python3 not available. Skipping correlation analysis."
        return 1
    fi
    
    # Create analysis script
    cat > /tmp/correlation_analysis.py << 'EOF'
import json
import yaml
import sys
import os
from pathlib import Path
import statistics

def load_experiment_data(results_dir):
    data = []
    
    for exp_dir in Path(results_dir).glob("mpc_ts*"):
        if exp_dir.is_dir():
            # Load experiment info
            info_file = exp_dir / "experiment_info.yaml"
            if not info_file.exists():
                continue
            
            try:
                with open(info_file, 'r') as f:
                    info = yaml.safe_load(f)
                
                # Find summary stats
                stats_files = list((exp_dir / "results").glob("summary_stats_*.json"))
                if not stats_files:
                    continue
                
                with open(stats_files[0], 'r') as f:
                    stats = json.load(f)
                
                # Extract parameters and metric
                params = info.get('parameters', {})
                metric_value = stats.get(sys.argv[2], {}).get('rms')
                
                if metric_value is not None:
                    data.append({
                        'name': exp_dir.name,
                        'metric': float(metric_value),
                        'target_speed': float(params.get('target_speed', 0)),
                        'horizon_length': int(params.get('horizon_length', 0)),
                        'control_dt': float(params.get('control_dt', 0)),
                        'position_weight': float(params.get('position_weight', 0)),
                        'yaw_weight': float(params.get('yaw_weight', 0)),
                        'control_weight': float(params.get('control_weight', 0)),
                        'max_steer_deg': float(params.get('max_steer_deg', 0)),
                        'path_type': params.get('path_type', 'unknown')
                    })
            except Exception as e:
                print(f"Error processing {exp_dir.name}: {e}", file=sys.stderr)
                continue
    
    return data

def calculate_correlation(x, y):
    """Calculate Pearson correlation coefficient"""
    if len(x) != len(y) or len(x) < 2:
        return 0.0
    
    mean_x = statistics.mean(x)
    mean_y = statistics.mean(y)
    
    numerator = sum((x[i] - mean_x) * (y[i] - mean_y) for i in range(len(x)))
    sum_sq_x = sum((x[i] - mean_x) ** 2 for i in range(len(x)))
    sum_sq_y = sum((y[i] - mean_y) ** 2 for i in range(len(y)))
    
    denominator = (sum_sq_x * sum_sq_y) ** 0.5
    
    if denominator == 0:
        return 0.0
    
    return numerator / denominator

def main():
    if len(sys.argv) != 3:
        print("Usage: python correlation_analysis.py RESULTS_DIR METRIC")
        sys.exit(1)
    
    results_dir = sys.argv[1]
    metric = sys.argv[2]
    
    data = load_experiment_data(results_dir)
    
    if not data:
        print("No valid experiment data found")
        return
    
    print(f"Analyzing {len(data)} experiments for {metric} correlations:\n")
    
    # Parameters to analyze
    numeric_params = [
        ('target_speed', 'Target Speed'),
        ('horizon_length', 'Horizon Length'),
        ('control_dt', 'Control DT'),
        ('position_weight', 'Position Weight'),
        ('yaw_weight', 'Yaw Weight'),
        ('control_weight', 'Control Weight'),
        ('max_steer_deg', 'Max Steer Degrees')
    ]
    
    metric_values = [d['metric'] for d in data]
    
    print("Parameter Correlations with", metric.replace('_', ' ').title(), ":")
    print("=" * 60)
    print(f"{'Parameter':<20} {'Correlation':<12} {'Interpretation'}")
    print("-" * 60)
    
    for param_key, param_name in numeric_params:
        param_values = [d[param_key] for d in data]
        correlation = calculate_correlation(param_values, metric_values)
        
        # Interpretation
        if abs(correlation) > 0.7:
            interpretation = "Strong"
        elif abs(correlation) > 0.5:
            interpretation = "Moderate"
        elif abs(correlation) > 0.3:
            interpretation = "Weak"
        else:
            interpretation = "Very Weak"
        
        direction = "negative" if correlation < 0 else "positive"
        interpretation += f" {direction}"
        
        print(f"{param_name:<20} {correlation:>8.3f}    {interpretation}")
    
    # Path type analysis
    print("\nPath Type Analysis:")
    print("=" * 40)
    path_groups = {}
    for d in data:
        path_type = d['path_type']
        if path_type not in path_groups:
            path_groups[path_type] = []
        path_groups[path_type].append(d['metric'])
    
    for path_type, values in path_groups.items():
        mean_val = statistics.mean(values)
        print(f"{path_type:<15}: {mean_val:.6f} (n={len(values)})")

if __name__ == "__main__":
    main()
EOF
    
    python3 /tmp/correlation_analysis.py "$dir" "$metric"
    rm -f /tmp/correlation_analysis.py
}

# Function to generate comparison plots
generate_plots() {
    local dir=$1
    local metric=$2
    
    log_info "=== Generating Comparison Plots ==="
    
    if ! command -v python3 &> /dev/null; then
        log_warning "Python3 not available. Skipping plot generation."
        return 1
    fi
    
    # Check if matplotlib is available
    if ! python3 -c "import matplotlib.pyplot as plt" 2>/dev/null; then
        log_warning "Matplotlib not available. Skipping plot generation."
        log_info "Install with: pip3 install matplotlib seaborn pandas"
        return 1
    fi
    
    log_info "Generating plots for $metric analysis..."
    
    # Create plotting script
    cat > /tmp/generate_plots.py << 'EOF'
import json
import yaml
import sys
import os
from pathlib import Path
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import numpy as np

def main():
    if len(sys.argv) != 3:
        print("Usage: python generate_plots.py RESULTS_DIR METRIC")
        sys.exit(1)
    
    results_dir = Path(sys.argv[1])
    metric = sys.argv[2]
    
    # Load data
    data = []
    for exp_dir in results_dir.glob("mpc_ts*"):
        if exp_dir.is_dir():
            info_file = exp_dir / "experiment_info.yaml"
            if not info_file.exists():
                continue
            
            try:
                with open(info_file, 'r') as f:
                    info = yaml.safe_load(f)
                
                stats_files = list((exp_dir / "results").glob("summary_stats_*.json"))
                if not stats_files:
                    continue
                
                with open(stats_files[0], 'r') as f:
                    stats = json.load(f)
                
                params = info.get('parameters', {})
                metric_value = stats.get(metric, {}).get('rms')
                
                if metric_value is not None:
                    data.append({
                        'experiment': exp_dir.name,
                        'metric_value': float(metric_value),
                        'target_speed': float(params.get('target_speed', 0)),
                        'horizon_length': int(params.get('horizon_length', 0)),
                        'control_dt': float(params.get('control_dt', 0)),
                        'position_weight': float(params.get('position_weight', 0)),
                        'yaw_weight': float(params.get('yaw_weight', 0)),
                        'control_weight': float(params.get('control_weight', 0)),
                        'max_steer_deg': float(params.get('max_steer_deg', 0)),
                        'path_type': params.get('path_type', 'unknown')
                    })
            except Exception as e:
                continue
    
    if not data:
        print("No valid data found for plotting")
        return
    
    df = pd.DataFrame(data)
    
    # Set up the plotting style
    plt.style.use('seaborn-v0_8' if 'seaborn-v0_8' in plt.style.available else 'default')
    
    # Create figure with subplots
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle(f'MPC Parameter Sweep Analysis - {metric.replace("_", " ").title()}', fontsize=16, fontweight='bold')
    
    # 1. Box plot by target speed
    sns.boxplot(data=df, x='target_speed', y='metric_value', ax=axes[0,0])
    axes[0,0].set_title('Performance by Target Speed')
    axes[0,0].set_xlabel('Target Speed (m/s)')
    axes[0,0].set_ylabel(metric.replace('_', ' ').title())
    
    # 2. Box plot by horizon length
    sns.boxplot(data=df, x='horizon_length', y='metric_value', ax=axes[0,1])
    axes[0,1].set_title('Performance by Horizon Length')
    axes[0,1].set_xlabel('Horizon Length')
    axes[0,1].set_ylabel(metric.replace('_', ' ').title())
    
    # 3. Box plot by path type
    sns.boxplot(data=df, x='path_type', y='metric_value', ax=axes[0,2])
    axes[0,2].set_title('Performance by Path Type')
    axes[0,2].set_xlabel('Path Type')
    axes[0,2].set_ylabel(metric.replace('_', ' ').title())
    
    # 4. Scatter plot: Position Weight vs Performance
    sns.scatterplot(data=df, x='position_weight', y='metric_value', hue='yaw_weight', ax=axes[1,0])
    axes[1,0].set_title('Position Weight vs Performance')
    axes[1,0].set_xlabel('Position Weight')
    axes[1,0].set_ylabel(metric.replace('_', ' ').title())
    
    # 5. Scatter plot: Control DT vs Performance
    sns.scatterplot(data=df, x='control_dt', y='metric_value', hue='target_speed', ax=axes[1,1])
    axes[1,1].set_title('Control DT vs Performance')
    axes[1,1].set_xlabel('Control DT (s)')
    axes[1,1].set_ylabel(metric.replace('_', ' ').title())
    
    # 6. Performance distribution
    axes[1,2].hist(df['metric_value'], bins=20, alpha=0.7, edgecolor='black')
    axes[1,2].set_title('Performance Distribution')
    axes[1,2].set_xlabel(metric.replace('_', ' ').title())
    axes[1,2].set_ylabel('Frequency')
    
    # Add statistics to the distribution plot
    mean_val = df['metric_value'].mean()
    median_val = df['metric_value'].median()
    axes[1,2].axvline(mean_val, color='red', linestyle='--', label=f'Mean: {mean_val:.4f}')
    axes[1,2].axvline(median_val, color='green', linestyle='--', label=f'Median: {median_val:.4f}')
    axes[1,2].legend()
    
    plt.tight_layout()
    
    # Save plot
    output_file = results_dir / f'parameter_analysis_{metric}.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"Plot saved to: {output_file}")
    
    # Create correlation heatmap
    fig2, ax = plt.subplots(1, 1, figsize=(10, 8))
    
    # Select numeric columns for correlation
    numeric_cols = ['metric_value', 'target_speed', 'horizon_length', 'control_dt', 
                   'position_weight', 'yaw_weight', 'control_weight', 'max_steer_deg']
    corr_df = df[numeric_cols].corr()
    
    sns.heatmap(corr_df, annot=True, cmap='coolwarm', center=0, ax=ax,
                square=True, fmt='.3f')
    ax.set_title(f'Parameter Correlation Matrix - {metric.replace("_", " ").title()}')
    
    plt.tight_layout()
    correlation_file = results_dir / f'correlation_matrix_{metric}.png'
    plt.savefig(correlation_file, dpi=300, bbox_inches='tight')
    print(f"Correlation matrix saved to: {correlation_file}")
    
    # Show summary statistics
    print(f"\nSummary Statistics for {metric}:")
    print("=" * 40)
    print(f"Count: {len(df)}")
    print(f"Mean: {df['metric_value'].mean():.6f}")
    print(f"Std: {df['metric_value'].std():.6f}")
    print(f"Min: {df['metric_value'].min():.6f}")
    print(f"Max: {df['metric_value'].max():.6f}")

if __name__ == "__main__":
    main()
EOF
    
    python3 /tmp/generate_plots.py "$dir" "$metric"
    rm -f /tmp/generate_plots.py
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -m|--metric)
            METRIC="$2"
            shift 2
            ;;
        -n|--top)
            TOP_N="$2"
            shift 2
            ;;
        -s|--summary)
            SHOW_SUMMARY=true
            shift
            ;;
        -b|--best)
            SHOW_BEST=true
            shift
            ;;
        -w|--worst)
            SHOW_WORST=true
            shift
            ;;
        -c|--correlations)
            SHOW_CORRELATIONS=true
            shift
            ;;
        -p|--plots)
            GENERATE_PLOTS=true
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        -*)
            log_error "Unknown option: $1"
            usage
            exit 1
            ;;
        *)
            if [[ -z "$RESULTS_DIR" ]]; then
                RESULTS_DIR="$1"
            else
                log_error "Multiple directories specified"
                usage
                exit 1
            fi
            shift
            ;;
    esac
done

# If no directory specified, try to find the latest one
if [[ -z "$RESULTS_DIR" ]]; then
    log_info "No results directory specified, looking for latest sweep..."
    if RESULTS_DIR=$(find_latest_sweep); then
        log_info "Found latest sweep: $RESULTS_DIR"
    else
        log_error "No results directory specified and no sweep directories found in ~/mpc_sweep_results"
        log_info "Run the parameter sweep script first or specify a results directory"
        exit 1
    fi
fi

# Expand tilde in path
RESULTS_DIR=$(eval echo "$RESULTS_DIR")

# Validate results directory
if ! validate_results_dir "$RESULTS_DIR"; then
    exit 1
fi

# Main analysis
log_info "Starting MPC Parameter Sweep Analysis"
log_info "Results Directory: $RESULTS_DIR"
log_info "Metric: $METRIC"
echo ""

# Show summary (always shown unless other specific options are used)
show_summary "$RESULTS_DIR"

# If only summary requested, exit here
if [[ "$SHOW_SUMMARY" == "true" ]]; then
    exit 0
fi

# Analyze by metric
if [[ "$SHOW_BEST" == "true" ]] || [[ "$SHOW_WORST" == "true" ]] || [[ "$SHOW_BEST" == "" && "$SHOW_WORST" == "" && "$SHOW_CORRELATIONS" == "" && "$GENERATE_PLOTS" == "" ]]; then
    analyze_by_metric "$RESULTS_DIR" "$METRIC" "$TOP_N" "$SHOW_BEST" "$SHOW_WORST"
fi

# Show correlations
if [[ "$SHOW_CORRELATIONS" == "true" ]]; then
    echo ""
    show_correlations "$RESULTS_DIR" "$METRIC"
fi

# Generate plots
if [[ "$GENERATE_PLOTS" == "true" ]]; then
    echo ""
    generate_plots "$RESULTS_DIR" "$METRIC"
fi

echo ""
log_success "Analysis complete!"
log_info "For detailed experiment results, check individual experiment directories in:"
log_info "  $RESULTS_DIR"

# Provide some helpful next steps
echo ""
echo "💡 NEXT STEPS:"
echo "1. Review individual experiment results:"
echo "   cd $RESULTS_DIR && find . -name 'EXPERIMENT_SUMMARY.md' | head -5 | xargs head"
echo ""
echo "2. Compare best vs worst experiments:"
echo "   $0 -b -w -n 3 $RESULTS_DIR"
echo ""
echo "3. Analyze different metrics:"
echo "   $0 -m heading_errors -c $RESULTS_DIR"
echo "   $0 -m control_smoothness -p $RESULTS_DIR"
echo ""
echo "4. Generate comprehensive analysis with plots:"
echo "   $0 -c -p $RESULTS_DIR"