import os
import json
import numpy as np
from ktune.core.utils import metrics

class DataLog:
    """Handles saving test data and metadata to files."""

    JOINT_NAMES = {
        11: "Left Shoulder Roll", 12: "Left Shoulder Pitch", 
        13: "Left Elbow Roll", 14: "Left Gripper",
        21: "Right Shoulder Roll", 22: "Right Shoulder Pitch",
        23: "Right Elbow Roll", 24: "Right Gripper",
        31: "Left Hip Yaw", 32: "Left Hip Roll",
        33: "Left Hip Pitch", 34: "Left Knee Pitch",
        35: "Left Ankle Pitch", 41: "Right Hip Yaw",
        42: "Right Hip Roll", 43: "Right Hip Pitch",
        44: "Right Knee Pitch", 45: "Right Ankle Pitch"
    }

    def __init__(self, config, sim_data=None, real_data=None):
        """Initialize the DataLogger.
        
        Args:
            config: Test configuration object
            sim_data (dict, optional): Simulation data
            real_data (dict, optional): Real robot data
        """
        self.config = config
        self.mode = config.mode
        self.sim_data = sim_data
        self.real_data = real_data

    def save_data(self, timestamp: str, data_dir: str):
        """Save test data to file.
        
        Args:
            timestamp (str): Timestamp for file naming
            data_dir (str): Directory to save data file
        """
        # Build data structure
        data = self._build_header(timestamp)
        
        # Only include data for active modes
        if self.config.mode in ['compare', 'sim'] and self.sim_data:
            data["sim_data"] = self.sim_data
        
        if self.config.mode in ['compare', 'real'] and self.real_data:
            data["real_data"] = self.real_data

        # Save to file
        filename = f"{timestamp}_{self.config.test}.json"
        filepath = os.path.join(data_dir, filename)
        
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)

    def _build_header(self, timestamp: str):
        """Build metadata header with all metrics."""
        joint_name = self.JOINT_NAMES.get(self.config.actuator_id, 
                                        f"id_{self.config.actuator_id}")

        # Build base header
        header = {
            "test_type": self.config.test,
            "mode": self.mode,
            "actuator_id": self.config.actuator_id,
            "joint_name": joint_name,
            "timestamp": timestamp,
            "robot_name": self.config.name,
            "start_position": self.config.start_pos,
            "sample_rate": self.config.sample_rate,
            "gains": {
                "sim": {"kp": self.config.sim_kp, "Kd": self.config.sim_kd},
                "real": {"kp": self.config.kp, "kd": self.config.kd, "ki": self.config.ki}
            },
            "acceleration": self.config.acceleration,
            "max_torque": self.config.max_torque,
            "torque_enabled": not self.config.torque_off
        }

        # Add tracking metrics and statistics
        tracking_metrics = {}
        data_statistics = {}

        # Only compute metrics for active modes with data
        if self.mode in ['compare', 'sim'] and self.sim_data:
            tracking_metrics["sim"] = metrics.compute_tracking_metrics(
                self.sim_data["cmd_time"], self.sim_data["cmd_pos"],
                self.sim_data["time"], self.sim_data["position"],
                self.sim_data["cmd_vel"], self.sim_data["velocity"]
            )
            data_statistics["sim"] = metrics.compute_data_statistics(
                self.sim_data["time"],
                self.sim_data["position"],
                self.sim_data["velocity"]
            )

        if self.mode in ['compare', 'real'] and self.real_data:
            tracking_metrics["real"] = metrics.compute_tracking_metrics(
                self.real_data["cmd_time"], self.real_data["cmd_pos"],
                self.real_data["time"], self.real_data["position"],
                self.real_data["cmd_vel"], self.real_data["velocity"]
            )
            data_statistics["real"] = metrics.compute_data_statistics(
                self.real_data["time"],
                self.real_data["position"],
                self.real_data["velocity"]
            )

        header.update({
            "tracking_metrics": tracking_metrics,
            "data_statistics": data_statistics
        })

        # Add test-specific metadata
        self._add_test_specific_metadata(header)
        
        return header

    def _add_test_specific_metadata(self, header):
        """Add metadata specific to test type."""
        if self.config.test == "chirp":
            header.update({
                "initial_frequency": self.config.chirp_init_freq,
                "sweep_rate": self.config.chirp_sweep_rate,
                "amplitude": self.config.chirp_amp,
                "duration": self.config.chirp_duration,
                "log_duration_pad": self.config.log_duration_pad,
                "total_duration": self.config.chirp_duration + self.config.log_duration_pad
            })
        elif self.config.test == "sine":
            header.update({
                "frequency": self.config.freq,
                "amplitude": self.config.amp,
                "duration": self.config.duration,
                "log_duration_pad": self.config.log_duration_pad,
                "total_duration": self.config.duration + self.config.log_duration_pad
            })
        elif self.config.test == "step":
            self._add_step_test_metadata(header)

    def _add_step_test_metadata(self, header):
        """Add step test specific metadata."""       
        vel = 0.0  # Default velocity limit
        step_metrics = {}

        if self.sim_data is not None:
            sim_metrics = metrics.compute_step_metrics(
                np.array(self.sim_data["time"]), 
                np.array(self.sim_data["position"]),
                self.config.step_size,
                self.config.step_hold_time,
                self.config.step_count
            )
            step_metrics["sim"] = self._compute_step_statistics(sim_metrics)

        if self.real_data is not None:
            real_metrics = metrics.compute_step_metrics(
                np.array(self.real_data["time"]), 
                np.array(self.real_data["position"]),
                self.config.step_size,
                self.config.step_hold_time,
                self.config.step_count
            )
            step_metrics["real"] = self._compute_step_statistics(real_metrics)

        header.update({
            "step_size": self.config.step_size,
            "step_hold_time": self.config.step_hold_time,
            "step_count": self.config.step_count,
            "velocity_limit": vel,
            "log_duration_pad": self.config.log_duration_pad,
            "total_duration": (self.config.step_hold_time * 
                             (2 * self.config.step_count + 1) + 
                             self.config.log_duration_pad),
            "step_metrics": step_metrics
        })

    def _compute_step_statistics(self, metrics_list):
        """Compute statistics from step metrics."""
        if not metrics_list:
            return None
        
        return {
            'max_overshoot': max(m['overshoot'] for m in metrics_list),
            'avg_overshoot': np.mean([m['overshoot'] for m in metrics_list]),
            'avg_rise_time': np.mean([m['rise_time'] for m in metrics_list if m['rise_time'] is not None]),
            'avg_settling_time': np.mean([m['settling_time'] for m in metrics_list if m['settling_time'] is not None]),
            'all_steps': metrics_list
        }

    def _prepare_output_data(self, header):
        """Prepare output data structures based on mode."""
        outputs = {}

        if self.sim_data is not None:
            outputs["sim"] = {
                "header": header,
                "data": {
                    "time": self.sim_data["time"],
                    "position": self.sim_data["position"],
                    "velocity": self.sim_data["velocity"],
                    "cmd_time": self.sim_data["cmd_time"],
                    "cmd_pos": self.sim_data["cmd_pos"],
                    "cmd_vel": self.sim_data["cmd_vel"]
                }
            }
            # Add frequency response data for chirp tests
            if self.config.test == "chirp" and "freq_response" in self.sim_data:
                outputs["sim"]["data"]["freq_response"] = self.sim_data["freq_response"]

        if self.real_data is not None:
            outputs["real"] = {
                "header": header,
                "data": {
                    "time": self.real_data["time"],
                    "position": self.real_data["position"],
                    "velocity": self.real_data["velocity"],
                    "cmd_time": self.real_data["cmd_time"],
                    "cmd_pos": self.real_data["cmd_pos"],
                    "cmd_vel": self.real_data["cmd_vel"]
                }
            }
            # Add frequency response data for chirp tests
            if self.config.test == "chirp" and "freq_response" in self.real_data:
                outputs["real"]["data"]["freq_response"] = self.real_data["freq_response"]

        return outputs

    def _save_to_files(self, timestamp: str, data_dir: str, outputs):
        """Save data to JSON files."""
        base_path = os.path.join(data_dir, f"{timestamp}_{self.config.test}")
        saved_files = []

        for system, data in outputs.items():
            filename = f"{base_path}_{system}.json"
            with open(filename, "w") as f:
                json.dump(data, f, indent=2)
            saved_files.append(filename)

        print(f"\nSaved data files:")
        for filename in saved_files:
            print(f"  {filename}")