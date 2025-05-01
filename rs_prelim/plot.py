import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import sys
import os
import datetime
import json
import glob
import argparse
def plot_actuator_data(file_path):

    # Read the JSON file
    with open(file_path, 'r') as f:
        json_data = json.load(f)
    
    meta_data = json_data['config']

    # Extract data points
    data_points = json_data['data']
    
    # Convert to DataFrame
    df = pd.DataFrame(data_points)
    
    # Create a figure with a specific size
    plt.figure(figsize=(12, 6))
    
    # Plot the position over time
    plt.plot(df['time_since_start'], df['position'], '-', linewidth=1)
    plt.plot(df['time_since_start'], df['commanded_position'], '--', linewidth=1)
    
    # Set labels and title
    plt.xlabel('Time')
    plt.ylabel('Position (deg)')
    plt.title(f'Actuator {meta_data["actuator_ids"]} Position over Time\n'
              f'{meta_data["step_hold_time"]}s step, {meta_data["step_size"]}° step size \n'
              f'{meta_data["step_count"]} steps, \n{meta_data["kp"]} kp, {meta_data["kd"]} kd, {meta_data["max_torque"]} max torque')

    plt.legend(['position', 'commanded position'])
    
    # Add grid
    plt.grid(True, alpha=0.3)
    
    # Adjust layout
    plt.tight_layout()
    
    # Generate timestamp for the output file
    curr_timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    plt.savefig(f'{curr_timestamp}.png')

if __name__ == "__main__":
    # If file path is provided as an argument, use it
    parser = argparse.ArgumentParser()
    parser.add_argument("--date", type=str, default=datetime.datetime.now().strftime("%Y%m%d"))
    args = parser.parse_args()

    if len(sys.argv) > 1:
        file_path = sys.argv[1]
    else:
        # Look for *_resp_step_* JSON files
        files = glob.glob(f'{args.date}/*_resp_step_*.json')
        
        if not files:
            # Fall back to other JSON files if none found
            files = glob.glob('*.json')
            
        if files:
            # Sort files by timestamp in the filename
            files.sort(key=lambda x: x.split('_')[-1].split('.')[0], reverse=True)
            file_path = files[0]
            print(f"Using newest file: {file_path}")
        else:
            print("No JSON data files found.")
            sys.exit(1)
    
    plot_actuator_data(file_path)
