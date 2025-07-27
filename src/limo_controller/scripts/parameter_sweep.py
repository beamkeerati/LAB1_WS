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
import psutil


class MPCParameterSweep(Node):
    """
    Automated parameter sweep experiment for MPC controller evaluation.
    
    This node systematically varies MPC parameters and evaluates performance
    sequentially - one experiment completes before the next begins.
    Properly manages process lifecycle to avoid multiple Gazebo instances.
    """
    
    def __init__(self):
        super().__init__('mpc_parameter_sweep')
        
        # Declare parameters for the sweep experiment
        self.declare_parameter('sweep_config_file', 'mpc_sweep_config.yaml')
        self.declare_parameter('results_directory', 'mpc_parameter_sweep')
        self.declare_parameter('experiment_duration', 30.0)  # Reduced duration for testing
        self.declare_parameter('timeout_duration', 50.0)  # Reduced timeout
        
        # Get parameters
        self.sweep_config_file = self.get_parameter('sweep_config_file').value
        self.results_directory = self.get_parameter('results_directory').value
        self.experiment_duration = self.get_parameter('experiment_duration').value
        self.timeout_duration = self.get_parameter('timeout_duration').value
        
        # Expand tilde in directory path
        self.results_directory = os.path.expanduser(self.results_directory)
        
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
        
        # Process management - ALL processes for current experiment
        self.sim_process = None
        self.mpc_process = None
        self.evaluator_process = None
        self.kinematics_processes = []
        
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
        
        # Start first experiment
        self.start_experiment_cycle()
        
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
        """Create default parameter sweep configuration - REDUCED for testing"""
        config = {
            'parameters': {
                'target_speed': {
                    'type': 'discrete',
                    'values': [0.5, 0.8]  # Reduced from [0.5, 0.8, 1.1]
                },
                'horizon_length': {
                    'type': 'discrete',
                    'values': [8, 12]  # Reduced from [8, 12, 16]
                },
                'control_dt': {
                    'type': 'discrete', 
                    'values': [0.1]  # Reduced from [0.05, 0.1]
                },
                'position_weight': {
                    'type': 'discrete',
                    'values': [5.0, 15.0]  # Reduced from [5.0, 15.0, 40.0]
                },
                'yaw_weight': {
                    'type': 'discrete',
                    'values': [5.0, 20.0]  # Reduced from [5.0, 20.0, 50.0]
                },
                'control_weight': {
                    'type': 'discrete',
                    'values': [0.1]  # Reduced from [0.1, 0.5]
                },
                'max_steer_deg': {
                    'type': 'discrete',
                    'values': [10.0]  # Reduced from [10.0, 15.0]
                },
                'path_type': {
                    'type': 'discrete',
                    'values': ["yaml", "switch_back"]
                }
            },
            'evaluation_metrics': [
                'position_errors',
                'heading_errors', 
                'cross_track_errors',
                'control_effort',
                'control_smoothness',
                'path_progress',
                'velocity_errors'
            ],
            'experiment_settings': {
                'warmup_time': 1.0,
                'evaluation_time': 20.0,  # Reduced from 35.0
                'cooldown_time': 1.0
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
    
    def start_experiment_cycle(self):
        """Start a complete experiment cycle"""
        if self.current_experiment_id >= len(self.parameter_combinations):
            self.get_logger().info("All experiments completed!")
            self.complete_parameter_sweep()
            return
        
        # Get current parameters
        params = self.parameter_combinations[self.current_experiment_id]
        experiment_name = f"sweep_exp_{self.current_experiment_id:03d}"
        
        self.get_logger().info(f"="*60)
        self.get_logger().info(f"Starting experiment {self.current_experiment_id + 1}/{len(self.parameter_combinations)}")
        self.get_logger().info(f"Experiment: {experiment_name}")
        self.get_logger().info(f"Parameters: {params}")
        self.get_logger().info(f"="*60)
        
        # Create experiment directory
        exp_dir = os.path.join(self.results_directory, experiment_name)
        os.makedirs(exp_dir, exist_ok=True)
        
        # Save experiment parameters
        with open(os.path.join(exp_dir, 'parameters.json'), 'w') as f:
            json.dump(params, f, indent=2)
        
        # Start all processes for this experiment
        success = self.launch_experiment_processes(exp_dir, experiment_name, params)
        
        if success:
            self.experiment_running = True
            self.experiment_start_time = time.time()
            self.get_logger().info(f"Experiment {self.current_experiment_id} launched successfully")
        else:
            self.get_logger().error(f"Failed to launch experiment {self.current_experiment_id}")
            self.finish_experiment_cycle(success=False)
    
    def launch_experiment_processes(self, exp_dir, experiment_name, params):
        """Launch all processes needed for one experiment"""
        try:
            # Step 1: Kill any existing Gazebo processes
            self.kill_existing_gazebo_processes()
            time.sleep(3.0)
            
            # Step 2: Launch Gazebo simulation
            self.get_logger().info("Launching Gazebo simulation...")
            self.sim_process = subprocess.Popen([
                'ros2', 'launch', 'limo_description', 'limo_ackerman_gz_path.launch.py'
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid)
            
            # Wait for simulation to initialize
            self.get_logger().info("Waiting for Gazebo to initialize...")
            time.sleep(15.0)
            
            # Step 3: Launch kinematics nodes
            self.get_logger().info("Launching kinematics nodes...")
            
            fk_process = subprocess.Popen([
                'ros2', 'run', 'limo_controller', 'forward_kinematics.py'
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid)
            
            ik_process = subprocess.Popen([
                'ros2', 'run', 'limo_controller', 'inverse_kinematics.py',
                '--ros-args', '-p', 'mode:=car'
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid)
            
            self.kinematics_processes = [fk_process, ik_process]
            
            # Wait for kinematics to initialize
            time.sleep(5.0)
            
            # Step 4: Launch MPC controller
            self.get_logger().info("Launching MPC controller...")
            mpc_cmd = [
                'ros2', 'run', 'limo_controller', 'mpc.py',
                '--ros-args',
                '-p', f'target_speed:={params.get("target_speed", 0.5)}',
                '-p', f'horizon_length:={params.get("horizon_length", 8)}',
                '-p', f'control_dt:={params.get("control_dt", 0.1)}',
                '-p', f'max_steer_deg:={params.get("max_steer_deg", 10.0)}',
                '-p', f'position_weight:={params.get("position_weight", 10.0)}',
                '-p', f'yaw_weight:={params.get("yaw_weight", 15.0)}',
                '-p', f'control_weight:={params.get("control_weight", 0.1)}',
                '-p', f'path_type:={params.get("path_type", "yaml")}',
                '-p', 'mode:=car'
            ]
            
            self.mpc_process = subprocess.Popen(
                mpc_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid)
            
            # Wait for MPC to initialize
            time.sleep(8.0)
            
            # Step 5: Launch evaluator
            self.get_logger().info("Launching evaluator...")
            self.evaluator_process = subprocess.Popen([
                'ros2', 'run', 'limo_controller', 'mpc_evaluator.py',
                '--ros-args',
                '-p', f'experiment_name:={experiment_name}',
                '-p', f'evaluation_duration:={self.experiment_duration - 5}',
                '-p', f'save_directory:={exp_dir}',
                '-p', 'enable_plots:=true',
                '-p', 'enable_real_time_plots:=false'
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid)
            
            # Wait for evaluator to start
            time.sleep(3.0)
            
            self.get_logger().info("All processes launched successfully")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error launching experiment processes: {e}")
            self.cleanup_experiment_processes()
            return False
    
    def kill_existing_gazebo_processes(self):
        """Kill any existing Gazebo and related processes"""
        processes_to_kill = ['gzserver', 'gzclient', 'gazebo', 'rviz2', 'ruby']
        
        for proc_name in processes_to_kill:
            try:
                for proc in psutil.process_iter(['pid', 'name']):
                    if proc_name in proc.info['name']:
                        self.get_logger().info(f"Killing existing {proc_name} process (PID: {proc.info['pid']})")
                        proc.kill()
                        proc.wait(timeout=3)
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.TimeoutExpired):
                pass
        
        # Additional cleanup
        time.sleep(2.0)
    
    def status_callback(self, msg):
        """Handle status messages from MPC controller"""
        try:
            status_data = json.loads(msg.data)
            status = status_data.get('status')
            
            if status == 'goal_reached':
                self.get_logger().info(f"✓ Experiment {self.current_experiment_id} completed successfully - goal reached")
                self.finish_experiment_cycle(success=True)
                
            elif status == 'experiment_failed':
                self.get_logger().warn(f"✗ Experiment {self.current_experiment_id} failed - MPC failure")
                self.finish_experiment_cycle(success=False)
                
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
            
            # Log progress every 10 seconds
            if int(elapsed_time) % 10 == 0 and int(elapsed_time) > 0:
                remaining_time = self.timeout_duration - elapsed_time
                self.get_logger().info(f"Experiment {self.current_experiment_id} running... {elapsed_time:.0f}s elapsed, {remaining_time:.0f}s remaining")
            
            if elapsed_time > self.timeout_duration:
                self.get_logger().warn(f"⏰ Experiment {self.current_experiment_id} timed out after {elapsed_time:.1f}s")
                self.finish_experiment_cycle(success=False)
    
    def finish_experiment_cycle(self, success=True):
        """Finish the current experiment cycle and cleanup"""
        if not self.experiment_running:
            return
        
        self.experiment_running = False
        
        self.get_logger().info(f"Finishing experiment {self.current_experiment_id} (success={success})")
        
        # Cleanup all processes
        self.cleanup_experiment_processes()
        
        # Wait for cleanup to complete
        time.sleep(5.0)
        
        # Parse and store results
        if success:
            result = self.parse_experiment_results()
            if result is not None:
                self.experiment_results.append(result)
                self.get_logger().info(f"✓ Successfully parsed results for experiment {self.current_experiment_id}")
            else:
                self.get_logger().warn(f"⚠ Failed to parse results for experiment {self.current_experiment_id}")
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
            self.get_logger().info(f"✗ Recorded failure for experiment {self.current_experiment_id}")
        
        # Move to next experiment
        self.current_experiment_id += 1
        
        # Wait between experiments for system to stabilize
        self.get_logger().info("Waiting between experiments for system cleanup...")
        time.sleep(10.0)
        
        # Start next experiment cycle
        self.start_experiment_cycle()
    
    def cleanup_experiment_processes(self):
        """Cleanup all processes from current experiment"""
        processes = [
            ("Evaluator", self.evaluator_process),
            ("MPC", self.mpc_process),
            ("Simulation", self.sim_process)
        ]
        
        # Add kinematics processes
        for i, proc in enumerate(self.kinematics_processes):
            processes.append((f"Kinematics-{i}", proc))
        
        # Terminate all processes
        for name, process in processes:
            if process is not None:
                try:
                    self.get_logger().info(f"Terminating {name} process...")
                    if hasattr(process, 'pid'):
                        # Kill entire process group
                        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    process.terminate()
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    self.get_logger().warn(f"Force killing {name} process...")
                    if hasattr(process, 'pid'):
                        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    process.kill()
                    process.wait()
                except Exception as e:
                    self.get_logger().warn(f"Error terminating {name}: {e}")
        
        # Reset process references
        self.sim_process = None
        self.mpc_process = None
        self.evaluator_process = None
        self.kinematics_processes = []
        
        # Additional cleanup of any remaining processes
        self.kill_existing_gazebo_processes()
    
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
        self.get_logger().info("🎉 Parameter sweep completed! Generating analysis...")
        
        # Final cleanup
        self.cleanup_experiment_processes()
        
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

The parameter sweep tested {len(self.parameter_combinations)} different parameter combinations.

## Results Summary

"""
        
        if successful_experiments > 0:
            df_success = df[df['success'] == True]
            
            # Performance summary
            if 'position_errors_rms' in df_success.columns:
                report += f"""
### Position Tracking Performance
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
        
        report += """
## Files Generated

- `sweep_results_*.csv` - Complete experimental data
- Individual experiment folders with detailed results

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