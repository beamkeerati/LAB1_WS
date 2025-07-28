#!/usr/bin/env python3

import yaml
import itertools
import subprocess
import time
import os
import signal
from typing import Dict, List, Any

# --- Configuration ---
# Path to your parameter sweep configuration file
CONFIG_FILE = 'src/limo_controller/config/mpc_sweep_config.yaml'

# ROS2 package and launch file to execute
ROS_PACKAGE = 'limo_controller'
LAUNCH_FILE = 'mpc_single_evaluation.launch.py'
# --- End Configuration ---

def parse_config(config_file: str) -> Dict[str, Any]:
    """Loads and parses the YAML configuration file."""
    try:
        with open(config_file, 'r') as f:
            return yaml.safe_load(f)
    except FileNotFoundError:
        print(f"Error: Configuration file not found at '{config_file}'")
        exit(1)
    except yaml.YAMLError as e:
        print(f"Error parsing YAML file: {e}")
        exit(1)

def generate_parameter_combinations(parameters: Dict[str, Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Generates all unique combinations of parameters."""
    if not parameters:
        return []

    param_names = list(parameters.keys())
    value_lists = [param['values'] for param in parameters.values()]

    # Create the Cartesian product of all parameter values
    product = list(itertools.product(*value_lists))

    # Convert the list of tuples to a list of dictionaries
    combinations = [dict(zip(param_names, values)) for values in product]
    return combinations

def run_experiment(combination: Dict[str, Any], settings: Dict[str, float], run_index: int, total_runs: int):
    """
    Constructs and runs a single experiment using ros2 launch.

    Args:
        combination (Dict[str, Any]): A dictionary of parameters for this run.
        settings (Dict[str, float]): Experiment settings like duration.
        run_index (int): The current run number.
        total_runs (int): The total number of runs.
    """
    # 1. Calculate total experiment duration
    warmup_time = settings.get('warmup_time', 2.0)
    evaluation_time = settings.get('evaluation_time', 60.0)
    cooldown_time = settings.get('cooldown_time', 2.0)
    total_duration = warmup_time + evaluation_time + cooldown_time

    # 2. Construct a unique experiment name to avoid overwriting results
    experiment_name = f"run_{run_index}_" + "_".join(f"{k}_{v}" for k, v in combination.items())
    experiment_name = experiment_name.replace('.', '_') # Sanitize name

    # 3. Build the ros2 launch command
    command = [
        'ros2', 'launch', ROS_PACKAGE, LAUNCH_FILE,
        f'experiment_name:={experiment_name}',
        f'duration:={evaluation_time}'
    ]
    for key, value in combination.items():
        command.append(f'{key}:={value}')

    print("-" * 80)
    print(f"🚀 Starting Run {run_index}/{total_runs}: {experiment_name}")
    print(f"   Parameters: {combination}")
    print(f"   Total Duration: {total_duration} seconds")
    print("-" * 80)

    # 4. Execute the command as a new process in its own process group
    #    preexec_fn=os.setsid allows us to kill the process and all its children
    process = None
    try:
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            preexec_fn=os.setsid
        )

        # 5. Wait for the specified duration
        time.sleep(total_duration)

    except KeyboardInterrupt:
        print("\n[INFO] Keyboard interrupt detected. Terminating all processes and exiting.")
        if process:
            # Kill the entire process group
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
        exit(0)
    except Exception as e:
        print(f"\n[ERROR] An error occurred while launching the process: {e}")
    finally:
        # 6. Terminate the process and all its children gracefully
        if process and process.poll() is None: # Check if process is still running
            print("\n[INFO] Time limit reached. Terminating process group...")
            # Send SIGINT (Ctrl+C) to the entire process group for a clean shutdown
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            
            # Wait for the process to terminate
            try:
                process.wait(timeout=15) # Add a timeout for waiting
                print("[SUCCESS] Process terminated successfully.")
            except subprocess.TimeoutExpired:
                print("[WARNING] Process did not terminate gracefully. Forcing kill.")
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        else:
             print("\n[INFO] Process already finished.")

        # Optional: capture and print output for debugging
        if process:
            stdout, stderr = process.communicate()
            if stdout:
                print("\n--- STDOUT ---")
                print(stdout)
            if stderr:
                print("\n--- STDERR ---")
                print(stderr)
        
        # Add a small delay to ensure all ROS nodes have shut down
        time.sleep(5)


if __name__ == '__main__':
    config = parse_config(CONFIG_FILE)
    param_combinations = generate_parameter_combinations(config.get('parameters', {}))
    experiment_settings = config.get('experiment_settings', {})
    
    total_runs = len(param_combinations)
    if total_runs == 0:
        print("No parameter combinations found in the config file. Exiting.")
        exit(0)

    print(f"🔥 Found {total_runs} parameter combinations to test. Starting sweep. 🔥")

    for i, combo in enumerate(param_combinations):
        run_experiment(combo, experiment_settings, i + 1, total_runs)

    print("-" * 80)
    print("✅ MPC parameter sweep completed.")
    print("-" * 80)