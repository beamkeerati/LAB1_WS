#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters, GetParameters
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import json
import os
import time
import subprocess
import signal
import yaml
from datetime import datetime
from itertools import product
import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor


class MPCParameterSweep(Node):
    """
    Automated parameter sweep experiment for MPC controller evaluation.
    
    This node systematically varies MPC parameters and evaluates performance
    to understand controller behavior and optimal parameter selection.
    """
    
    def __init__(self):
        super().__init__('mpc_parameter_sweep')
        
        # Declare parameters for the sweep experiment
        self.declare_parameter('sweep_config_file', 'mpc_sweep_config.yaml')
        self.declare_parameter('results_directory', '/tmp/mpc_parameter_sweep')
        self.declare_parameter('experiment_duration', 60.0)  # Duration per experiment
        self.declare_parameter('max_concurrent_experiments', 1)  # Safety: run one at a time
        self.declare_parameter('enable_parallel', False)  # Enable parallel execution
        
        # Get parameters
        self.sweep_config_file = self.get_parameter('sweep_config_file').value
        self.results_directory = self.get_parameter('results_directory').value
        self.experiment_duration = self.get_parameter('experiment_duration').value
        self.max_concurrent = self.get_parameter('max_concurrent_experiments').value
        self.enable_parallel = self.get_parameter('enable_parallel').value
        
        # Create results directory
        os.makedirs(self.results_directory, exist_ok=True)
        
        # Load sweep configuration
        self.sweep_config = self.load_sweep_config()
        
        # Generate parameter combinations
        self.parameter_combinations = self.generate_parameter_combinations()
        
        # Results storage
        self.experiment_results = []
        
        self.get_logger().info(f"Parameter sweep initialized")
        self.get_logger().info(f"Total experiments: {len(self.parameter_combinations)}")
        self.get_logger().info(f"Estimated duration: {len(self.parameter_combinations) * self.experiment_duration / 60:.1f} minutes")
        
        # Start the parameter sweep
        self.run_parameter_sweep()
        
    def load_sweep_config(self):
        """Load parameter sweep configuration"""
        try:
            with open(self.sweep_config_file, 'r') as f:
                config = yaml.safe_load(f)
        except FileNotFoundError:
            # Create default configuration if file doesn't exist
            config = self.create_default_sweep_config()
            with open(self.sweep_config_file, 'w') as f:
                yaml.dump(config, f, default_flow_style=False)
            self.get_logger().info(f"Created default sweep config: {self.sweep_config_file}")
        
        return config
    
    def create_default_sweep_config(self):
        """Create default parameter sweep configuration"""
        config = {
            'parameters': {
                'target_speed': {
                    'type': 'linear',
                    'min': 0.3,
                    'max': 1.2,
                    'steps': 4
                },
                'horizon_length': {
                    'type': 'discrete',
                    'values': [8, 10, 15, 20]
                },
                'control_dt': {
                    'type': 'discrete', 
                    'values': [0.05, 0.1, 0.2]
                },
                'position_weight': {
                    'type': 'logarithmic',
                    'min': 1.0,
                    'max': 50.0,
                    'steps': 4
                },
                'yaw_weight': {
                    'type': 'logarithmic',
                    'min': 1.0,
                    'max': 50.0,
                    'steps': 4
                },
                'max_steer_deg': {
                    'type': 'linear',
                    'min': 5.0,
                    'max': 15.0,
                    'steps': 3
                }
            },
            'evaluation_metrics': [
                'position_errors',
                'heading_errors', 
                'cross_track_errors',
                'control_effort',
                'control_smoothness',
                'path_progress'
            ],
            'experiment_settings': {
                'warmup_time': 5.0,
                'evaluation_time': 50.0,
                'cooldown_time': 5.0
            }
        }
        
        return config
    
    def generate_parameter_combinations(self):
        """Generate all parameter combinations for the sweep"""
        param_ranges = {}
        
        for param_name, param_config in self.sweep_config['parameters'].items():
            if param_config['type'] == 'linear':
                param_ranges[param_name] = np.linspace(
                    param_config['min'], 
                    param_config['max'], 
                    param_config['steps']
                )
            elif param_config['type'] == 'logarithmic':
                param_ranges[param_name] = np.logspace(
                    np.log10(param_config['min']), 
                    np.log10(param_config['max']), 
                    param_config['steps']
                )
            elif param_config['type'] == 'discrete':
                param_ranges[param_name] = param_config['values']
            else:
                raise ValueError(f"Unknown parameter type: {param_config['type']}")
        
        # Generate all combinations
        param_names = list(param_ranges.keys())
        param_values = list(param_ranges.values())
        
        combinations = []
        for combination in product(*param_values):
            param_dict = dict(zip(param_names, combination))
            combinations.append(param_dict)
        
        return combinations
    
    def run_parameter_sweep(self):
        """Execute the parameter sweep experiment"""
        self.get_logger().info("Starting parameter sweep...")
        
        if self.enable_parallel and self.max_concurrent > 1:
            self.run_parallel_sweep()
        else:
            self.run_sequential_sweep()
        
        # Generate comprehensive analysis
        self.generate_sweep_analysis()
        
        self.get_logger().info("Parameter sweep completed!")
        
    def run_sequential_sweep(self):
        """Run experiments sequentially"""
        total_experiments = len(self.parameter_combinations)
        
        for i, params in enumerate(self.parameter_combinations):
            self.get_logger().info(f"Running experiment {i+1}/{total_experiments}")
            self.get_logger().info(f"Parameters: {params}")
            
            try:
                # Run single experiment
                result = self.run_single_experiment(params, i)
                if result is not None:
                    self.experiment_results.append(result)
                else:
                    self.get_logger().warn(f"Experiment {i+1} failed")
                    
            except Exception as e:
                self.get_logger().error(f"Error in experiment {i+1}: {e}")
                continue
            
            # Small delay between experiments
            time.sleep(2.0)
    
    def run_parallel_sweep(self):
        """Run experiments in parallel (use with caution)"""
        self.get_logger().info(f"Running {self.max_concurrent} experiments in parallel")
        
        with ProcessPoolExecutor(max_workers=self.max_concurrent) as executor:
            futures = []
            
            for i, params in enumerate(self.parameter_combinations):
                future = executor.submit(self.run_single_experiment, params, i)
                futures.append(future)
            
            # Collect results
            for i, future in enumerate(futures):
                try:
                    result = future.result(timeout=self.experiment_duration + 30)
                    if result is not None:
                        self.experiment_results.append(result)
                    else:
                        self.get_logger().warn(f"Experiment {i+1} failed")
                except Exception as e:
                    self.get_logger().error(f"Error in parallel experiment {i+1}: {e}")
    
    def run_single_experiment(self, parameters, experiment_id):
        """Run a single parameter experiment"""
        experiment_name = f"sweep_exp_{experiment_id:03d}"
        
        # Create experiment-specific directory
        exp_dir = os.path.join(self.results_directory, experiment_name)
        os.makedirs(exp_dir, exist_ok=True)
        
        # Save experiment parameters
        with open(os.path.join(exp_dir, 'parameters.json'), 'w') as f:
            json.dump(parameters, f, indent=2)
        
        try:
            # Launch MPC controller with specific parameters
            mpc_process = self.launch_mpc_controller(parameters)
            
            # Launch evaluator
            evaluator_process = self.launch_evaluator(experiment_name, exp_dir)
            
            # Wait for experiment to complete
            time.sleep(self.experiment_duration)
            
            # Terminate processes
            self.terminate_process(mpc_process)
            self.terminate_process(evaluator_process)
            
            # Wait a bit for files to be written
            time.sleep(2.0)
            
            # Load and parse results
            result = self.parse_experiment_results(exp_dir, parameters, experiment_id)
            
            return result
            
        except Exception as e:
            self.get_logger().error(f"Error running experiment {experiment_id}: {e}")
            return None
    
    def launch_mpc_controller(self, parameters):
        """Launch MPC controller with specified parameters"""
        cmd = [
            'ros2', 'run', 'limo_controller', 'mpc.py',
            '--ros-args',
            '-p', f'target_speed:={parameters.get("target_speed", 0.8)}',
            '-p', f'horizon_length:={int(parameters.get("horizon_length", 10))}',
            '-p', f'control_dt:={parameters.get("control_dt", 0.1)}',
            '-p', f'max_steer_deg:={parameters.get("max_steer_deg", 10.0)}'
        ]
        
        # Add position and yaw weights if they exist (these would need to be added to your MPC node)
        if 'position_weight' in parameters:
            cmd.extend(['-p', f'position_weight:={parameters["position_weight"]}'])
        if 'yaw_weight' in parameters:
            cmd.extend(['-p', f'yaw_weight:={parameters["yaw_weight"]}'])
        
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        # Give it time to start
        time.sleep(3.0)
        
        return process
    
    def launch_evaluator(self, experiment_name, save_dir):
        """Launch performance evaluator"""
        cmd = [
            'ros2', 'run', 'limo_controller', 'mpc_evaluator.py',
            '--ros-args',
            '-p', f'experiment_name:={experiment_name}',
            '-p', f'evaluation_duration:={self.experiment_duration - 10}',  # Slightly shorter
            '-p', f'save_directory:={save_dir}',
            '-p', 'enable_plots:=true',
            '-p', 'enable_real_time_plots:=false'
        ]
        
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        # Give it time to start
        time.sleep(2.0)
        
        return process
    
    def terminate_process(self, process):
        """Safely terminate a process"""
        if process is None:
            return
            
        try:
            # Try graceful termination first
            process.terminate()
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            # Force kill if necessary
            process.kill()
            process.wait()
        except Exception as e:
            self.get_logger().warn(f"Error terminating process: {e}")
    
    def parse_experiment_results(self, exp_dir, parameters, experiment_id):
        """Parse results from an experiment"""
        try:
            # Find the most recent summary statistics file
            summary_files = [f for f in os.listdir(exp_dir) if f.startswith('summary_stats_') and f.endswith('.json')]
            
            if not summary_files:
                self.get_logger().warn(f"No summary statistics found for experiment {experiment_id}")
                return None
            
            # Use the most recent file
            summary_file = sorted(summary_files)[-1]
            
            with open(os.path.join(exp_dir, summary_file), 'r') as f:
                stats = json.load(f)
            
            # Extract key metrics
            result = {
                'experiment_id': experiment_id,
                'parameters': parameters.copy(),
                'metrics': {}
            }
            
            # Extract relevant statistics for each metric
            for metric_name in self.sweep_config['evaluation_metrics']:
                if metric_name in stats:
                    metric_stats = stats[metric_name]
                    result['metrics'][metric_name] = {
                        'mean': metric_stats.get('mean', 0),
                        'std': metric_stats.get('std', 0),
                        'rms': metric_stats.get('rms', 0),
                        'max': metric_stats.get('max', 0),
                        'min': metric_stats.get('min', 0)
                    }
            
            return result
            
        except Exception as e:
            self.get_logger().error(f"Error parsing results for experiment {experiment_id}: {e}")
            return None
    
    def generate_sweep_analysis(self):
        """Generate comprehensive analysis of parameter sweep results"""
        if not self.experiment_results:
            self.get_logger().error("No experiment results to analyze")
            return
        
        self.get_logger().info("Generating parameter sweep analysis...")
        
        # Convert results to DataFrame for analysis
        df = self.results_to_dataframe()
        
        # Save raw results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        df.to_csv(os.path.join(self.results_directory, f'sweep_results_{timestamp}.csv'), index=False)
        
        # Generate analysis plots
        self.generate_parameter_correlation_plots(df, timestamp)
        self.generate_sensitivity_analysis(df, timestamp)
        self.generate_pareto_analysis(df, timestamp)
        self.generate_parameter_heatmaps(df, timestamp)
        
        # Generate recommendations
        self.generate_parameter_recommendations(df, timestamp)
        
        self.get_logger().info(f"Analysis complete! Results saved to {self.results_directory}")
    
    def results_to_dataframe(self):
        """Convert experiment results to pandas DataFrame"""
        rows = []
        
        for result in self.experiment_results:
            row = {'experiment_id': result['experiment_id']}
            
            # Add parameters
            for param_name, param_value in result['parameters'].items():
                row[f'param_{param_name}'] = param_value
            
            # Add metrics
            for metric_name, metric_stats in result['metrics'].items():
                for stat_name, stat_value in metric_stats.items():
                    row[f'{metric_name}_{stat_name}'] = stat_value
            
            rows.append(row)
        
        return pd.DataFrame(rows)
    
    def generate_parameter_correlation_plots(self, df, timestamp):
        """Generate correlation plots between parameters and performance"""
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        fig.suptitle('Parameter vs Performance Correlation Analysis', fontsize=16)
        
        # Key metrics to analyze
        key_metrics = [
            'position_errors_rms',
            'heading_errors_rms', 
            'cross_track_errors_rms',
            'control_effort_mean',
            'control_smoothness_rms',
            'path_progress_max'
        ]
        
        param_cols = [col for col in df.columns if col.startswith('param_')]
        
        for i, metric in enumerate(key_metrics):
            if metric not in df.columns:
                continue
                
            ax = axes[i//3, i%3]
            
            # Create correlation plot for each parameter
            for param in param_cols:
                if param in df.columns:
                    ax.scatter(df[param], df[metric], label=param.replace('param_', ''), alpha=0.6)
            
            ax.set_xlabel('Parameter Value')
            ax.set_ylabel(metric.replace('_', ' ').title())
            ax.set_title(f'{metric.replace("_", " ").title()} vs Parameters')
            ax.legend()
            ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.results_directory, f'parameter_correlations_{timestamp}.png'), 
                   dpi=300, bbox_inches='tight')
        plt.close()
    
    def generate_sensitivity_analysis(self, df, timestamp):
        """Generate parameter sensitivity analysis"""
        param_cols = [col for col in df.columns if col.startswith('param_')]
        metric_cols = [col for col in df.columns if 'rms' in col or 'mean' in col]
        
        # Calculate sensitivity (normalized correlation coefficients)
        sensitivity_matrix = np.zeros((len(param_cols), len(metric_cols)))
        
        for i, param in enumerate(param_cols):
            for j, metric in enumerate(metric_cols):
                if param in df.columns and metric in df.columns:
                    corr = df[param].corr(df[metric])
                    sensitivity_matrix[i, j] = abs(corr) if not np.isnan(corr) else 0
        
        # Create heatmap
        fig, ax = plt.subplots(figsize=(12, 8))
        sns.heatmap(sensitivity_matrix, 
                   xticklabels=[col.replace('_', ' ').title() for col in metric_cols],
                   yticklabels=[col.replace('param_', '').replace('_', ' ').title() for col in param_cols],
                   annot=True, cmap='RdYlBu_r', ax=ax)
        
        ax.set_title('Parameter Sensitivity Analysis\n(Absolute Correlation with Performance Metrics)')
        plt.tight_layout()
        plt.savefig(os.path.join(self.results_directory, f'sensitivity_analysis_{timestamp}.png'), 
                   dpi=300, bbox_inches='tight')
        plt.close()
    
    def generate_pareto_analysis(self, df, timestamp):
        """Generate Pareto frontier analysis for multi-objective optimization"""
        # Use position error and control effort as competing objectives
        if 'position_errors_rms' in df.columns and 'control_effort_mean' in df.columns:
            
            fig, ax = plt.subplots(figsize=(10, 8))
            
            scatter = ax.scatter(df['position_errors_rms'], df['control_effort_mean'], 
                               c=df['experiment_id'], cmap='viridis', alpha=0.7)
            
            # Find Pareto frontier (simplified approach)
            pareto_indices = self.find_pareto_frontier(
                df['position_errors_rms'].values, 
                df['control_effort_mean'].values
            )
            
            if len(pareto_indices) > 0:
                pareto_points = df.iloc[pareto_indices]
                ax.plot(pareto_points['position_errors_rms'], 
                       pareto_points['control_effort_mean'], 
                       'r-', linewidth=2, label='Pareto Frontier')
                ax.scatter(pareto_points['position_errors_rms'], 
                          pareto_points['control_effort_mean'], 
                          c='red', s=100, marker='*', label='Pareto Optimal')
            
            ax.set_xlabel('Position Error RMS (m)')
            ax.set_ylabel('Control Effort Mean')
            ax.set_title('Pareto Analysis: Tracking Accuracy vs Control Effort')
            ax.legend()
            ax.grid(True, alpha=0.3)
            
            plt.colorbar(scatter, label='Experiment ID')
            plt.tight_layout()
            plt.savefig(os.path.join(self.results_directory, f'pareto_analysis_{timestamp}.png'), 
                       dpi=300, bbox_inches='tight')
            plt.close()
    
    def find_pareto_frontier(self, obj1, obj2):
        """Find Pareto frontier for two objectives (both to be minimized)"""
        n_points = len(obj1)
        pareto_indices = []
        
        for i in range(n_points):
            is_pareto = True
            for j in range(n_points):
                if i != j:
                    # Check if point j dominates point i
                    if obj1[j] <= obj1[i] and obj2[j] <= obj2[i] and (obj1[j] < obj1[i] or obj2[j] < obj2[i]):
                        is_pareto = False
                        break
            
            if is_pareto:
                pareto_indices.append(i)
        
        return pareto_indices
    
    def generate_parameter_heatmaps(self, df, timestamp):
        """Generate heatmaps for parameter interactions"""
        param_cols = [col for col in df.columns if col.startswith('param_')]
        
        if len(param_cols) < 2:
            return
        
        # Focus on key performance metric
        performance_metric = 'position_errors_rms'
        
        if performance_metric not in df.columns:
            return
        
        # Generate heatmaps for parameter pairs
        n_params = len(param_cols)
        n_pairs = min(6, n_params * (n_params - 1) // 2)  # Limit to 6 pairs
        
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        fig.suptitle(f'Parameter Interaction Heatmaps - {performance_metric.replace("_", " ").title()}', fontsize=16)
        
        pair_count = 0
        for i in range(n_params):
            for j in range(i + 1, n_params):
                if pair_count >= 6:
                    break
                
                param1, param2 = param_cols[i], param_cols[j]
                ax = axes[pair_count // 3, pair_count % 3]
                
                # Create pivot table for heatmap
                try:
                    # Discretize continuous parameters for heatmap
                    df_temp = df.copy()
                    df_temp[f'{param1}_binned'] = pd.cut(df_temp[param1], bins=5, labels=False)
                    df_temp[f'{param2}_binned'] = pd.cut(df_temp[param2], bins=5, labels=False)
                    
                    pivot_table = df_temp.pivot_table(
                        values=performance_metric,
                        index=f'{param1}_binned',
                        columns=f'{param2}_binned',
                        aggfunc='mean'
                    )
                    
                    sns.heatmap(pivot_table, annot=True, fmt='.3f', cmap='RdYlBu_r', ax=ax)
                    ax.set_title(f'{param1.replace("param_", "")} vs {param2.replace("param_", "")}')
                    ax.set_xlabel(param2.replace('param_', '').replace('_', ' ').title())
                    ax.set_ylabel(param1.replace('param_', '').replace('_', ' ').title())
                    
                except Exception as e:
                    # Fallback to scatter plot if heatmap fails
                    scatter = ax.scatter(df[param2], df[param1], c=df[performance_metric], cmap='RdYlBu_r')
                    ax.set_xlabel(param2.replace('param_', '').replace('_', ' ').title())
                    ax.set_ylabel(param1.replace('param_', '').replace('_', ' ').title())
                    ax.set_title(f'{param1.replace("param_", "")} vs {param2.replace("param_", "")}')
                    plt.colorbar(scatter, ax=ax)
                
                pair_count += 1
        
        # Hide empty subplots
        for idx in range(pair_count, 6):
            axes[idx // 3, idx % 3].set_visible(False)
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.results_directory, f'parameter_heatmaps_{timestamp}.png'), 
                   dpi=300, bbox_inches='tight')
        plt.close()
    
    def generate_parameter_recommendations(self, df, timestamp):
        """Generate parameter tuning recommendations"""
        # Find best performing parameter sets
        key_metrics = ['position_errors_rms', 'heading_errors_rms', 'cross_track_errors_rms']
        
        recommendations = {
            'best_overall': {},
            'best_tracking': {},
            'best_stability': {},
            'parameter_insights': {},
            'trade_offs': {}
        }
        
        # Best overall (weighted combination)
        if all(metric in df.columns for metric in key_metrics):
            # Normalize metrics and compute weighted score
            df_norm = df.copy()
            weights = {'position_errors_rms': 0.4, 'heading_errors_rms': 0.3, 'cross_track_errors_rms': 0.3}
            
            combined_score = 0
            for metric, weight in weights.items():
                metric_norm = (df[metric] - df[metric].min()) / (df[metric].max() - df[metric].min())
                combined_score += weight * metric_norm
            
            best_idx = combined_score.idxmin()
            recommendations['best_overall'] = {
                'experiment_id': df.loc[best_idx, 'experiment_id'],
                'parameters': {col.replace('param_', ''): df.loc[best_idx, col] 
                             for col in df.columns if col.startswith('param_')},
                'performance': {metric: df.loc[best_idx, metric] for metric in key_metrics}
            }
        
        # Best tracking accuracy
        if 'position_errors_rms' in df.columns:
            best_tracking_idx = df['position_errors_rms'].idxmin()
            recommendations['best_tracking'] = {
                'experiment_id': df.loc[best_tracking_idx, 'experiment_id'],
                'parameters': {col.replace('param_', ''): df.loc[best_tracking_idx, col] 
                             for col in df.columns if col.startswith('param_')},
                'position_error': df.loc[best_tracking_idx, 'position_errors_rms']
            }
        
        # Best stability (lowest control variation)
        if 'control_smoothness_rms' in df.columns:
            best_stability_idx = df['control_smoothness_rms'].idxmin()
            recommendations['best_stability'] = {
                'experiment_id': df.loc[best_stability_idx, 'experiment_id'],
                'parameters': {col.replace('param_', ''): df.loc[best_stability_idx, col] 
                             for col in df.columns if col.startswith('param_')},
                'control_smoothness': df.loc[best_stability_idx, 'control_smoothness_rms']
            }
        
        # Parameter insights (correlation analysis)
        param_cols = [col for col in df.columns if col.startswith('param_')]
        for param in param_cols:
            param_name = param.replace('param_', '')
            insights = {}
            
            for metric in key_metrics:
                if metric in df.columns:
                    corr = df[param].corr(df[metric])
                    if not np.isnan(corr):
                        if abs(corr) > 0.3:  # Significant correlation
                            insights[metric] = {
                                'correlation': corr,
                                'interpretation': 'increases' if corr > 0 else 'decreases'
                            }
            
            if insights:
                recommendations['parameter_insights'][param_name] = insights
        
        # Trade-off analysis
        if 'position_errors_rms' in df.columns and 'control_effort_mean' in df.columns:
            # Find parameters that balance accuracy and control effort
            pos_norm = (df['position_errors_rms'] - df['position_errors_rms'].min()) / \
                      (df['position_errors_rms'].max() - df['position_errors_rms'].min())
            ctrl_norm = (df['control_effort_mean'] - df['control_effort_mean'].min()) / \
                       (df['control_effort_mean'].max() - df['control_effort_mean'].min())
            
            balance_score = pos_norm + ctrl_norm
            balanced_idx = balance_score.idxmin()
            
            recommendations['trade_offs']['balanced_performance'] = {
                'experiment_id': df.loc[balanced_idx, 'experiment_id'],
                'parameters': {col.replace('param_', ''): df.loc[balanced_idx, col] 
                             for col in df.columns if col.startswith('param_')},
                'position_error': df.loc[balanced_idx, 'position_errors_rms'],
                'control_effort': df.loc[balanced_idx, 'control_effort_mean']
            }
        
        # Save recommendations
        with open(os.path.join(self.results_directory, f'parameter_recommendations_{timestamp}.json'), 'w') as f:
            json.dump(recommendations, f, indent=2)
        
        # Generate readable report
        self.generate_recommendations_report(recommendations, timestamp)
        
        return recommendations
    
    def generate_recommendations_report(self, recommendations, timestamp):
        """Generate human-readable recommendations report"""
        report = f"""
# MPC Parameter Tuning Recommendations

**Generated:** {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
**Total Experiments:** {len(self.experiment_results)}

## Best Parameter Sets

### Best Overall Performance
"""
        
        if 'best_overall' in recommendations and recommendations['best_overall']:
            best = recommendations['best_overall']
            report += f"""
**Experiment ID:** {best['experiment_id']}

**Parameters:**
"""
            for param, value in best['parameters'].items():
                report += f"- {param.replace('_', ' ').title()}: {value:.4f}\n"
            
            report += "\n**Performance:**\n"
            for metric, value in best['performance'].items():
                report += f"- {metric.replace('_', ' ').title()}: {value:.4f}\n"
        
        if 'best_tracking' in recommendations and recommendations['best_tracking']:
            best_track = recommendations['best_tracking']
            report += f"""
### Best Tracking Accuracy
**Experiment ID:** {best_track['experiment_id']}
**Position Error:** {best_track['position_error']:.4f} m

**Parameters:**
"""
            for param, value in best_track['parameters'].items():
                report += f"- {param.replace('_', ' ').title()}: {value:.4f}\n"
        
        if 'best_stability' in recommendations and recommendations['best_stability']:
            best_stable = recommendations['best_stability']
            report += f"""
### Best Control Stability
**Experiment ID:** {best_stable['experiment_id']}
**Control Smoothness:** {best_stable['control_smoothness']:.4f}

**Parameters:**
"""
            for param, value in best_stable['parameters'].items():
                report += f"- {param.replace('_', ' ').title()}: {value:.4f}\n"
        
        # Parameter insights
        if 'parameter_insights' in recommendations:
            report += """
## Parameter Insights

### Significant Parameter Effects
"""
            for param, insights in recommendations['parameter_insights'].items():
                report += f"\n**{param.replace('_', ' ').title()}:**\n"
                for metric, effect in insights.items():
                    direction = "worsens" if effect['interpretation'] == 'increases' else "improves"
                    report += f"- {direction} {metric.replace('_', ' ')} (correlation: {effect['correlation']:.3f})\n"
        
        # Trade-offs
        if 'trade_offs' in recommendations and 'balanced_performance' in recommendations['trade_offs']:
            balanced = recommendations['trade_offs']['balanced_performance']
            report += f"""
## Balanced Performance Recommendation

For applications requiring a balance between tracking accuracy and control effort:

**Experiment ID:** {balanced['experiment_id']}
**Position Error:** {balanced['position_error']:.4f} m
**Control Effort:** {balanced['control_effort']:.4f}

**Parameters:**
"""
            for param, value in balanced['parameters'].items():
                report += f"- {param.replace('_', ' ').title()}: {value:.4f}\n"
        
        report += """
## General Tuning Guidelines

1. **For Better Tracking:** Increase position and yaw weights, consider longer horizon
2. **For Smoother Control:** Increase control cost weights, reduce target speed
3. **For Faster Response:** Decrease control time step, increase steering limits
4. **For Robustness:** Use moderate parameter values, avoid extremes

## Files Generated

- `sweep_results_*.csv` - Raw experimental data
- `parameter_correlations_*.png` - Parameter vs performance plots
- `sensitivity_analysis_*.png` - Parameter sensitivity heatmap
- `pareto_analysis_*.png` - Multi-objective optimization analysis
- `parameter_heatmaps_*.png` - Parameter interaction effects
- `parameter_recommendations_*.json` - Detailed recommendations data

---
*Generated by MPC Parameter Sweep Tool*
"""
        
        # Save the report
        with open(os.path.join(self.results_directory, f'tuning_recommendations_{timestamp}.md'), 'w') as f:
            f.write(report)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        sweep_node = MPCParameterSweep()
        # The node completes its work in __init__ and then exits
        
    except KeyboardInterrupt:
        print("Parameter sweep interrupted by user")
    except Exception as e:
        print(f"Error in parameter sweep: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()