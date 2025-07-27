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
from std_msgs.msg import String


class MPCParameterSweep(Node):
    """
    Automated parameter sweep experiment for MPC controller evaluation.
    
    This node systematically varies MPC parameters and evaluates performance
    sequentially - one experiment completes before the next begins.
    """
    
    def __init__(self):
        super().__init__('mpc_parameter_sweep')
        
        # Declare parameters for the sweep experiment
        self.declare_parameter('sweep_config_file', 'mpc_sweep_config.yaml')
        self.declare_parameter('results_directory', '~/mpc_parameter_sweep')
        self.declare_parameter('experiment_duration', 60.0)  # Max duration per experiment
        self.declare_parameter('timeout_duration', 80.0)  # Timeout if experiment hangs
        
        # Get parameters
        self.sweep_config_file = self.get_parameter('sweep_config_file').value
        self.results_directory = self.get_parameter('results_directory').value
        self.experiment_duration = self.get_parameter('experiment_duration').value
        self.timeout_duration = self.get_parameter('timeout_duration').value
        
        # Create results directory
        os.makedirs(self.results_directory, exist_ok=True)
        
        # Load sweep configuration
        self.sweep_config = self.load_sweep_config()
        
        # Generate parameter combinations
        self.parameter_combinations = self.generate_parameter_combinations()
        
        # Results storage
        self.experiment_results = []
        
        # Experiment control
        self.current_experiment_id = 0
        self.experiment_running = False
        self.experiment_start_time = None
        self.mpc_process = None
        self.evaluator_process = None
        
        # Publishers and subscribers for communication with MPC node
        self.param_update_pub = self.create_publisher(String, "/mpc/parameter_update", 10)
        self.status_sub = self.create_subscription(
            String, "/mpc/experiment_status", self.status_callback, 10
        )
        
        # Timer for experiment management
        self.experiment_timer = self.create_timer(2.0, self.experiment_management_callback)
        
        self.get_logger().info(f"Parameter sweep initialized")
        self.get_logger().info(f"Total experiments: {len(self.parameter_combinations)}")
        self.get_logger().info(f"Estimated duration: {len(self.parameter_combinations) * self.experiment_duration / 60:.1f} minutes")
        
        # Start the parameter sweep
        self.start_simulation_processes()
        
    def load_sweep_config(self):
        """Load parameter sweep configuration"""
        try:
            config_path = os.path.join(
                os.path.dirname(os.path.dirname(__file__)), 
                'config', 
                self.sweep_config_file
            )
            
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                
            self.get_logger().info(f"Loaded sweep config from: {config_path}")
            return config
            
        except FileNotFoundError:
            # Create default configuration if file doesn't exist
            config = self.create_default_sweep_config()
            config_path = os.path.join(
                os.path.dirname(os.path.dirname(__file__)), 
                'config', 
                self.sweep_config_file
            )
            os.makedirs(os.path.dirname(config_path), exist_ok=True)
            with open(config_path, 'w') as f:
                yaml.dump(config, f, default_flow_style=False)
            self.get_logger().info(f"Created default sweep config: {config_path}")
            return config
    
    def create_default_sweep_config(self):
        """Create default parameter sweep configuration"""
        config = {
            'parameters': {
                'target_speed': {
                    'type': 'linear',
                    'min': 0.4,
                    'max': 1.0,
                    'steps': 3
                },
                'horizon_length': {
                    'type': 'discrete',
                    'values': [8, 12, 16]
                },
                'control_dt': {
                    'type': 'discrete', 
                    'values': [0.05, 0.1]
                },
                'position_weight': {
                    'type': 'logarithmic',
                    'min': 5.0,
                    'max': 50.0,
                    'steps': 3
                },
                'yaw_weight': {
                    'type': 'logarithmic',
                    'min': 5.0,
                    'max': 50.0,
                    'steps': 3
                },
                'max_steer_deg': {
                    'type': 'linear',
                    'min': 8.0,
                    'max': 15.0,
                    'steps': 2
                },
                'path_type': {
                    'type': 'discrete',
                    'values': ["straight", "forward", "switch_back"]
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
                'warmup_time': 3.0,
                'evaluation_time': 45.0,
                'cooldown_time': 2.0
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
    
    def start_simulation_processes(self):
        """Start the simulation and MPC processes"""
        try:
            # Launch Gazebo simulation
            self.get_logger().info("Starting Gazebo simulation...")
            self.sim_process = subprocess.Popen([
                'ros2', 'launch', 'limo_description', 'limo_ackerman_gz_path.launch.py'
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            
            # Wait for simulation to start
            time.sleep(10.0)
            
            # Launch MPC controller
            self.get_logger().info("Starting MPC controller...")
            self.mpc_process = subprocess.Popen([
                'ros2', 'run', 'limo_controller', 'mpc.py'
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            
            # Wait for MPC controller to initialize
            time.sleep(5.0)
            
            # Start first experiment
            self.start_next_experiment()
            
        except Exception as e:
            self.get_logger().error(f"Error starting simulation processes: {e}")
    
    def start_next_experiment(self):
        """Start the next experiment in the sequence"""
        if self.current_experiment_id >= len(self.parameter_combinations):
            self.get_logger().info("All experiments completed!")
            self.complete_parameter_sweep()
            return
        
        # Get current parameters
        params = self.parameter_combinations[self.current_experiment_id]
        experiment_name = f"sweep_exp_{self.current_experiment_id:03d}"
        
        self.get_logger().info(f"Starting experiment {self.current_experiment_id + 1}/{len(self.parameter_combinations)}")
        self.get_logger().info(f"Parameters: {params}")
        
        # Create experiment directory
        exp_dir = os.path.join(self.results_directory, experiment_name)
        os.makedirs(exp_dir, exist_ok=True)
        
        # Save experiment parameters
        with open(os.path.join(exp_dir, 'parameters.json'), 'w') as f:
            json.dump(params, f, indent=2)
        
        try:
            # Launch evaluator for this experiment
            self.evaluator_process = subprocess.Popen([
                'ros2', 'run', 'limo_controller', 'mpc_evaluator.py',
                '--ros-args',
                '-p', f'experiment_name:={experiment_name}',
                '-p', f'evaluation_duration:={self.experiment_duration - 5}',
                '-p', f'save_directory:={exp_dir}',
                '-p', 'enable_plots:=true',
                '-p', 'enable_real_time_plots:=false'
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            
            # Wait a moment for evaluator to start
            time.sleep(2.0)
            
            # Send parameter update to MPC controller
            self.send_parameter_update(params)
            
            # Set experiment state
            self.experiment_running = True
            self.experiment_start_time = time.time()
            
        except Exception as e:
            self.get_logger().error(f"Error starting experiment {self.current_experiment_id}: {e}")
            self.finish_current_experiment(success=False)
    
    def send_parameter_update(self, parameters):
        """Send parameter update message to MPC controller"""
        try:
            update_msg = String()
            update_data = {
                'action': 'update_parameters',
                'parameters': parameters
            }
            update_msg.data = json.dumps(update_data)
            self.param_update_pub.publish(update_msg)
            self.get_logger().info("Parameter update sent to MPC controller")
            
        except Exception as e:
            self.get_logger().error(f"Error sending parameter update: {e}")
    
    def status_callback(self, msg):
        """Handle status messages from MPC controller"""
        try:
            status_data = json.loads(msg.data)
            status = status_data.get('status')
            
            if status == 'goal_reached':
                self.get_logger().info(f"Experiment {self.current_experiment_id} completed successfully - goal reached")
                self.finish_current_experiment(success=True)
                
            elif status == 'experiment_failed':
                self.get_logger().warn(f"Experiment {self.current_experiment_id} failed - MPC failure")
                self.finish_current_experiment(success=False)
                
            elif status == 'parameters_updated':
                self.get_logger().info(f"Parameters updated for experiment {self.current_experiment_id}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing status message: {e}")
    
    def experiment_management_callback(self):
        """Manage experiment timeouts and progression"""
        if not self.experiment_running:
            return
        
        # Check for timeout
        if self.experiment_start_time is not None:
            elapsed_time = time.time() - self.experiment_start_time
            
            if elapsed_time > self.timeout_duration:
                self.get_logger().warn(f"Experiment {self.current_experiment_id} timed out after {elapsed_time:.1f}s")
                self.finish_current_experiment(success=False)
    
    def finish_current_experiment(self, success=True):
        """Finish the current experiment and prepare for the next"""
        if not self.experiment_running:
            return
        
        self.experiment_running = False
        
        # Terminate evaluator process
        if self.evaluator_process is not None:
            try:
                self.evaluator_process.terminate()
                self.evaluator_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.evaluator_process.kill()
                self.evaluator_process.wait()
            except Exception as e:
                self.get_logger().warn(f"Error terminating evaluator: {e}")
            finally:
                self.evaluator_process = None
        
        # Wait for files to be written
        time.sleep(3.0)
        
        # Parse and store results
        if success:
            result = self.parse_experiment_results()
            if result is not None:
                self.experiment_results.append(result)
            else:
                self.get_logger().warn(f"Failed to parse results for experiment {self.current_experiment_id}")
        else:
            # Create a failure record
            params = self.parameter_combinations[self.current_experiment_id]
            result = {
                'experiment_id': self.current_experiment_id,
                'parameters': params.copy(),
                'success': False,
                'metrics': {}
            }
            self.experiment_results.append(result)
        
        # Move to next experiment
        self.current_experiment_id += 1
        
        # Wait between experiments
        time.sleep(5.0)
        
        # Start next experiment
        self.start_next_experiment()
    
    def parse_experiment_results(self):
        """Parse results from the current experiment"""
        try:
            experiment_name = f"sweep_exp_{self.current_experiment_id:03d}"
            exp_dir = os.path.join(self.results_directory, experiment_name)
            
            # Find the most recent summary statistics file
            summary_files = [f for f in os.listdir(exp_dir) 
                           if f.startswith('summary_stats_') and f.endswith('.json')]
            
            if not summary_files:
                self.get_logger().warn(f"No summary statistics found for experiment {self.current_experiment_id}")
                return None
            
            # Use the most recent file
            summary_file = sorted(summary_files)[-1]
            
            with open(os.path.join(exp_dir, summary_file), 'r') as f:
                stats = json.load(f)
            
            # Extract key metrics
            params = self.parameter_combinations[self.current_experiment_id]
            result = {
                'experiment_id': self.current_experiment_id,
                'parameters': params.copy(),
                'success': True,
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
            self.get_logger().error(f"Error parsing results for experiment {self.current_experiment_id}: {e}")
            return None
    
    def complete_parameter_sweep(self):
        """Complete the parameter sweep and generate analysis"""
        self.get_logger().info("Parameter sweep completed! Generating analysis...")
        
        # Terminate processes
        if self.mpc_process is not None:
            try:
                self.mpc_process.terminate()
                self.mpc_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.mpc_process.kill()
                self.mpc_process.wait()
            except Exception as e:
                self.get_logger().warn(f"Error terminating MPC process: {e}")
        
        if hasattr(self, 'sim_process') and self.sim_process is not None:
            try:
                self.sim_process.terminate()
                self.sim_process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                self.sim_process.kill()
                self.sim_process.wait()
            except Exception as e:
                self.get_logger().warn(f"Error terminating simulation: {e}")
        
        # Generate comprehensive analysis
        if self.experiment_results:
            self.generate_sweep_analysis()
        else:
            self.get_logger().error("No experiment results to analyze")
        
        self.get_logger().info("Parameter sweep analysis completed!")
        
        # Shutdown the node
        self.destroy_node()
        rclpy.shutdown()
    
    def generate_sweep_analysis(self):
        """Generate comprehensive analysis of parameter sweep results"""
        self.get_logger().info("Generating parameter sweep analysis...")
        
        # Convert results to DataFrame for analysis
        df = self.results_to_dataframe()
        
        # Save raw results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        df.to_csv(os.path.join(self.results_directory, f'sweep_results_{timestamp}.csv'), index=False)
        
        # Generate analysis plots
        self.generate_parameter_correlation_plots(df, timestamp)
        self.generate_sensitivity_analysis(df, timestamp)
        self.generate_path_comparison_plots(df, timestamp)
        
        # Generate recommendations
        self.generate_parameter_recommendations(df, timestamp)
        
        # Generate summary report
        self.generate_summary_report(df, timestamp)
        
        self.get_logger().info(f"Analysis complete! Results saved to {self.results_directory}")
    
    def results_to_dataframe(self):
        """Convert experiment results to pandas DataFrame"""
        rows = []
        
        for result in self.experiment_results:
            row = {
                'experiment_id': result['experiment_id'],
                'success': result.get('success', True)
            }
            
            # Add parameters
            for param_name, param_value in result['parameters'].items():
                row[f'param_{param_name}'] = param_value
            
            # Add metrics (only for successful experiments)
            if result.get('success', True) and 'metrics' in result:
                for metric_name, metric_stats in result['metrics'].items():
                    for stat_name, stat_value in metric_stats.items():
                        row[f'{metric_name}_{stat_name}'] = stat_value
            
            rows.append(row)
        
        return pd.DataFrame(rows)
    
    def generate_parameter_correlation_plots(self, df, timestamp):
        """Generate correlation plots between parameters and performance"""
        # Filter only successful experiments
        df_success = df[df['success'] == True].copy()
        
        if len(df_success) == 0:
            self.get_logger().warn("No successful experiments for correlation analysis")
            return
        
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
        
        param_cols = [col for col in df_success.columns if col.startswith('param_') and col != 'param_path_type']
        
        for i, metric in enumerate(key_metrics):
            if metric not in df_success.columns:
                continue
                
            ax = axes[i//3, i%3]
            
            # Create correlation plot for each parameter
            for param in param_cols:
                if param in df_success.columns:
                    ax.scatter(df_success[param], df_success[metric], 
                             label=param.replace('param_', ''), alpha=0.6)
            
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
        # Filter only successful experiments
        df_success = df[df['success'] == True].copy()
        
        if len(df_success) == 0:
            self.get_logger().warn("No successful experiments for sensitivity analysis")
            return
        
        param_cols = [col for col in df_success.columns if col.startswith('param_') and col != 'param_path_type']
        metric_cols = [col for col in df_success.columns if 'rms' in col or 'mean' in col]
        
        if len(param_cols) == 0 or len(metric_cols) == 0:
            return
        
        # Calculate sensitivity (normalized correlation coefficients)
        sensitivity_matrix = np.zeros((len(param_cols), len(metric_cols)))
        
        for i, param in enumerate(param_cols):
            for j, metric in enumerate(metric_cols):
                if param in df_success.columns and metric in df_success.columns:
                    corr = df_success[param].corr(df_success[metric])
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
    
    def generate_path_comparison_plots(self, df, timestamp):
        """Generate plots comparing performance across different path types"""
        # Filter only successful experiments
        df_success = df[df['success'] == True].copy()
        
        if 'param_path_type' not in df_success.columns:
            return
        
        # Key metrics for comparison
        metrics = ['position_errors_rms', 'heading_errors_rms', 'cross_track_errors_rms']
        
        fig, axes = plt.subplots(1, 3, figsize=(18, 6))
        fig.suptitle('Performance Comparison Across Path Types', fontsize=16)
        
        for i, metric in enumerate(metrics):
            if metric in df_success.columns:
                # Box plot for each path type
                path_types = df_success['param_path_type'].unique()
                data_for_boxplot = [df_success[df_success['param_path_type'] == pt][metric].values 
                                  for pt in path_types]
                
                axes[i].boxplot(data_for_boxplot, labels=path_types)
                axes[i].set_title(metric.replace('_', ' ').title())
                axes[i].set_ylabel('Error Value')
                axes[i].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.results_directory, f'path_comparison_{timestamp}.png'), 
                   dpi=300, bbox_inches='tight')
        plt.close()
    
    def generate_parameter_recommendations(self, df, timestamp):
        """Generate parameter tuning recommendations"""
        # Filter only successful experiments
        df_success = df[df['success'] == True].copy()
        
        if len(df_success) == 0:
            recommendations = {'message': 'No successful experiments to analyze'}
        else:
            recommendations = self._analyze_successful_experiments(df_success)
        
        # Save recommendations
        with open(os.path.join(self.results_directory, f'parameter_recommendations_{timestamp}.json'), 'w') as f:
            json.dump(recommendations, f, indent=2)
        
        return recommendations
    
    def _analyze_successful_experiments(self, df_success):
        """Analyze successful experiments for recommendations"""
        recommendations = {
            'best_overall': {},
            'best_by_path': {},
            'parameter_insights': {},
            'success_rate': {}
        }
        
        # Calculate success rate by parameter
        total_experiments = len(self.experiment_results)
        successful_experiments = len(df_success)
        recommendations['success_rate']['overall'] = successful_experiments / total_experiments
        
        # Best overall performance
        key_metrics = ['position_errors_rms', 'heading_errors_rms', 'cross_track_errors_rms']
        
        if all(metric in df_success.columns for metric in key_metrics):
            # Normalize metrics and compute weighted score
            df_norm = df_success.copy()
            weights = {'position_errors_rms': 0.4, 'heading_errors_rms': 0.3, 'cross_track_errors_rms': 0.3}
            
            combined_score = 0
            for metric, weight in weights.items():
                metric_range = df_success[metric].max() - df_success[metric].min()
                if metric_range > 0:
                    metric_norm = (df_success[metric] - df_success[metric].min()) / metric_range
                    combined_score += weight * metric_norm
            
            best_idx = combined_score.idxmin()
            recommendations['best_overall'] = {
                'experiment_id': int(df_success.loc[best_idx, 'experiment_id']),
                'parameters': {col.replace('param_', ''): df_success.loc[best_idx, col] 
                             for col in df_success.columns if col.startswith('param_')},
                'performance': {metric: float(df_success.loc[best_idx, metric]) for metric in key_metrics}
            }
        
        # Best by path type
        if 'param_path_type' in df_success.columns and 'position_errors_rms' in df_success.columns:
            for path_type in df_success['param_path_type'].unique():
                path_data = df_success[df_success['param_path_type'] == path_type]
                if len(path_data) > 0:
                    best_idx = path_data['position_errors_rms'].idxmin()
                    recommendations['best_by_path'][path_type] = {
                        'experiment_id': int(path_data.loc[best_idx, 'experiment_id']),
                        'parameters': {col.replace('param_', ''): path_data.loc[best_idx, col] 
                                     for col in path_data.columns if col.startswith('param_')},
                        'position_error': float(path_data.loc[best_idx, 'position_errors_rms'])
                    }
        
        return recommendations
    
    def generate_summary_report(self, df, timestamp):
        """Generate a comprehensive summary report"""
        total_experiments = len(self.parameter_combinations)
        successful_experiments = len(df[df['success'] == True])
        failed_experiments = total_experiments - successful_experiments
        
        report = f"""
# MPC Parameter Sweep Summary Report

**Generated:** {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
**Total Experiments:** {total_experiments}
**Successful Experiments:** {successful_experiments} ({successful_experiments/total_experiments*100:.1f}%)
**Failed Experiments:** {failed_experiments} ({failed_experiments/total_experiments*100:.1f}%)

## Experiment Overview

The parameter sweep tested {len(self.parameter_combinations)} different parameter combinations across:
"""
        
        for param_name, param_config in self.sweep_config['parameters'].items():
            if param_config['type'] == 'discrete':
                values = param_config['values']
                report += f"- **{param_name.replace('_', ' ').title()}:** {len(values)} values ({values})\n"
            else:
                steps = param_config.get('steps', 'N/A')
                min_val = param_config.get('min', 'N/A')
                max_val = param_config.get('max', 'N/A')
                report += f"- **{param_name.replace('_', ' ').title()}:** {steps} steps from {min_val} to {max_val}\n"
        
        if successful_experiments > 0:
            df_success = df[df['success'] == True]
            
            # Performance summary
            if 'position_errors_rms' in df_success.columns:
                report += f"""
## Performance Summary (Successful Experiments Only)

### Position Tracking
- **Best Position Error:** {df_success['position_errors_rms'].min():.4f} m
- **Worst Position Error:** {df_success['position_errors_rms'].max():.4f} m
- **Average Position Error:** {df_success['position_errors_rms'].mean():.4f} m
"""
            
            if 'param_path_type' in df_success.columns:
                report += "\n### Performance by Path Type\n"
                for path_type in df_success['param_path_type'].unique():
                    path_data = df_success[df_success['param_path_type'] == path_type]
                    count = len(path_data)
                    if 'position_errors_rms' in path_data.columns:
                        avg_error = path_data['position_errors_rms'].mean()
                        report += f"- **{path_type.title()}:** {count} experiments, avg error: {avg_error:.4f} m\n"
        
        # Failure analysis
        if failed_experiments > 0:
            df_failed = df[df['success'] == False]
            report += f"""
## Failure Analysis

{failed_experiments} experiments failed to complete successfully. Common failure modes may include:
- MPC solver convergence issues
- Robot getting stuck or lost
- Simulation instabilities
- Parameter combinations that are infeasible

"""
            
            # Analyze failure patterns by parameter if possible
            if 'param_path_type' in df_failed.columns:
                report += "### Failures by Path Type:\n"
                for path_type in df_failed['param_path_type'].unique():
                    path_failures = len(df_failed[df_failed['param_path_type'] == path_type])
                    report += f"- **{path_type.title()}:** {path_failures} failures\n"
        
        report += """
## Files Generated

- `sweep_results_*.csv` - Complete experimental data
- `parameter_correlations_*.png` - Parameter vs performance analysis
- `sensitivity_analysis_*.png` - Parameter sensitivity heatmap
- `path_comparison_*.png` - Performance comparison across path types
- `parameter_recommendations_*.json` - Best parameter sets
- Individual experiment folders with detailed results

## Next Steps

1. Review the best performing parameter sets in the recommendations file
2. Examine correlation plots to understand parameter effects
3. Consider running additional experiments around promising parameter regions
4. Test the recommended parameters in real-world scenarios

---
*Generated by MPC Parameter Sweep Tool*
"""
        
        # Save the report
        with open(os.path.join(self.results_directory, f'sweep_summary_{timestamp}.md'), 'w') as f:
            f.write(report)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        sweep_node = MPCParameterSweep()
        rclpy.spin(sweep_node)
        
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