import pandas as pd
import matplotlib.pyplot as plt
import sys
import os
import json
import glob
import argparse
import numpy as np

def load_data_from_file(file_path):
    """Load data from a JSON file and return a DataFrame with the data."""
    with open(file_path, 'r') as f:
        json_data = json.load(f)
    
    # Extract metadata and data points
    meta_data = json_data['config']
    data_points = json_data['data']
    
    # Convert to DataFrame
    df = pd.DataFrame(data_points)
    
    return df, meta_data

def plot_actuator_subplots(actuator_files,
                           output_file: str = "actuator_subplots.png"):
    """Create a vertical stack of sub-plots, one per actuator."""
    # Keep only actuator IDs that have BOTH real & sim files
    valid_ids = [aid for aid, f in actuator_files.items()
                 if f['real'] and f['sim']]
    if not valid_ids:
        print("No valid actuator data found.")
        return

    # ------------------------------------------------------------------
    n = len(valid_ids)
    fig, axes = plt.subplots(n, 1, figsize=(18, 4 * n), sharex=True)

    if n == 1:                       # make iterable if only one subplot
        axes = [axes]
    # ------------------------------------------------------------------

    for ax, actuator_id in zip(axes, sorted(valid_ids)):
        files = actuator_files[actuator_id]
        real_df, real_meta = load_data_from_file(files['real'])
        sim_df, sim_meta = load_data_from_file(files['sim'])

        ax.plot(real_df["time_since_start"], real_df["commanded_position"],
                "k:",  linewidth=1,  label="Commanded")
        ax.plot(real_df["time_since_start"], real_df["position"],
                "r-", linewidth=1.2, label="Real")
        ax.plot(sim_df["time_since_start"],  sim_df["position"],
                "b--", linewidth=1.2, label="Sim")

        ax.set_ylabel("Pos (deg)", fontsize=11)
        
        # Updated title with armature and frictionloss from sim model
        ax.set_title(f"Actuator {actuator_id} - Kp={real_meta['kp']}, Kd={real_meta['kd']}, MaxTq={real_meta['max_torque']}\n"
                     f"Sim params: Armature={sim_meta['armature']}, Friction={sim_meta['frictionloss']}, Damping={sim_meta['damping']}, Actuatorfrcrange={sim_meta['actuatorfrcrange']}",
                     fontsize=12)
        
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=10)

    axes[-1].set_xlabel("Time (s)", fontsize=11)
    fig.suptitle("Real vs Simulation – Step Response", fontsize=18)
    fig.tight_layout(rect=[0, 0, 1, 0.99])
    fig.savefig(output_file)
    print(f"Plot saved -> {output_file}")

def main():
    parser = argparse.ArgumentParser(
        description="Overlay real & sim data for each actuator")
    parser.add_argument("--date", default="20250502",
                        help="Folder date (YYYYMMDD)")
    args = parser.parse_args()

    # Build path INSIDE rs_prelim
    date_dir = os.path.join( args.date)
    if not os.path.isdir(date_dir):
        print(f"Directory '{date_dir}' not found.")
        sys.exit(1)

    files = glob.glob(os.path.join(date_dir, "*.json"))
    if not files:
        print(f"No JSON files in '{date_dir}'.")
        sys.exit(1)

    # Group by actuator
    actuator_files = {}
    for fp in files:
        mode, act_id, *_ = os.path.basename(fp).split("_")
        actuator_files.setdefault(act_id, {"real": None, "sim": None})
        actuator_files[act_id][mode] = fp

    # Create sub-plots
    plot_actuator_subplots(actuator_files)

if __name__ == "__main__":
    main()
