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

    def __init__(self, config, sim_data, real_data):
        """Initialize the DataLogger.
        
        Args:
            config: Test configuration object
            sim_data (dict): Simulation data
            real_data (dict): Real robot data
        """
        self.config = config
        self.sim_data = sim_data
        self.real_data = real_data

    def save_data(self, timestamp: str, data_dir: str):
        """Save test data and metadata to JSON files.
        
        Args:
            timestamp (str): Timestamp for file naming
            data_dir (str): Directory to save files
        """
        header = self._build_header(timestamp)
        sim_output, real_output = self._prepare_output_data(header)
        self._save_to_files(timestamp, data_dir, sim_output, real_output)

    def _build_header(self, timestamp: str):
        """Build metadata header with all metrics."""
        joint_name = self.JOINT_NAMES.get(self.config.actuator_id, 
                                        f"id_{self.config.actuator_id}")

        # Build base header
        header = {
            "test_type": self.config.test,
            "actuator_id": self.config.actuator_id,
            "joint_name": joint_name,
            "timestamp": timestamp,
            "robot_name": self.config.name,
            "start_position": self.config.start_pos,
            "sample_rate": self.config.sample_rate,
            "gains": {
                "sim": {"kp": self.config.sim_kp, "kv": self.config.sim_kv},
                "real": {"kp": self.config.kp, "kd": self.config.kd, "ki": self.config.ki}
            },
            "acceleration": self.config.acceleration,
            "max_torque": self.config.max_torque,
            "torque_enabled": not self.config.torque_off
        }

        # Add tracking metrics and statistics
        header.update({
            "tracking_metrics": {
                "sim": metrics.compute_tracking_metrics(
                    self.sim_data["cmd_time"], self.sim_data["cmd_pos"],
                    self.sim_data["time"], self.sim_data["position"],
                    self.sim_data["cmd_vel"], self.sim_data["velocity"]
                ),
                "real": metrics.compute_tracking_metrics(
                    self.real_data["cmd_time"], self.real_data["cmd_pos"],
                    self.real_data["time"], self.real_data["position"],
                    self.real_data["cmd_vel"], self.real_data["velocity"]
                )
            },
            "data_statistics": {
                "sim": metrics.compute_data_statistics(
                    self.sim_data["time"],
                    self.sim_data["position"],
                    self.sim_data["velocity"]
                ),
                "real": metrics.compute_data_statistics(
                    self.real_data["time"],
                    self.real_data["position"],
                    self.real_data["velocity"]
                )
            }
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
        sim_metrics = metrics.compute_step_metrics(
            np.array(self.sim_data["time"]), 
            np.array(self.sim_data["position"]),
            self.config.step_size,
            self.config.step_hold_time,
            self.config.step_count
        )
        real_metrics = metrics.compute_step_metrics(
            np.array(self.real_data["time"]), 
            np.array(self.real_data["position"]),
            self.config.step_size,
            self.config.step_hold_time,
            self.config.step_count
        )

        # Calculate average and max metrics
        sim_stats = {
            'max_overshoot': max(m['overshoot'] for m in sim_metrics) if sim_metrics else 0.0,
            'avg_overshoot': np.mean([m['overshoot'] for m in sim_metrics]) if sim_metrics else 0.0,
            'avg_rise_time': np.mean([m['rise_time'] for m in sim_metrics if m['rise_time'] is not None]),
            'avg_settling_time': np.mean([m['settling_time'] for m in sim_metrics if m['settling_time'] is not None])
        }
        real_stats = {
            'max_overshoot': max(m['overshoot'] for m in real_metrics) if real_metrics else 0.0,
            'avg_overshoot': np.mean([m['overshoot'] for m in real_metrics]) if real_metrics else 0.0,
            'avg_rise_time': np.mean([m['rise_time'] for m in real_metrics if m['rise_time'] is not None]),
            'avg_settling_time': np.mean([m['settling_time'] for m in real_metrics if m['settling_time'] is not None])
        }

        header.update({
            "step_size": self.config.step_size,
            "step_hold_time": self.config.step_hold_time,
            "step_count": self.config.step_count,
            "velocity_limit": vel,
            "log_duration_pad": self.config.log_duration_pad,
            "total_duration": (self.config.step_hold_time * 
                             (2 * self.config.step_count + 1) + 
                             self.config.log_duration_pad),
            "sim_metrics": {
                "max_overshoot": sim_stats['max_overshoot'],
                "avg_overshoot": sim_stats['avg_overshoot'],
                "avg_rise_time": sim_stats['avg_rise_time'],
                "avg_settling_time": sim_stats['avg_settling_time'],
                "all_steps": sim_metrics  # Store metrics for each step
            },
            "real_metrics": {
                "max_overshoot": real_stats['max_overshoot'],
                "avg_overshoot": real_stats['avg_overshoot'],
                "avg_rise_time": real_stats['avg_rise_time'],
                "avg_settling_time": real_stats['avg_settling_time'],
                "all_steps": real_metrics  # Store metrics for each step
            }
        })

    def _prepare_output_data(self, header):
        """Prepare sim and real output data structures."""
        sim_output = {
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

        real_output = {
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
        if self.config.test == "chirp" and "freq_response" in self.sim_data:
            sim_output["data"]["freq_response"] = self.sim_data["freq_response"]
            real_output["data"]["freq_response"] = self.real_data["freq_response"]

        return sim_output, real_output

    def _save_to_files(self, timestamp: str, data_dir: str, sim_output, real_output):
        """Save data to JSON files."""
        base_path = os.path.join(data_dir, f"{timestamp}_{self.config.test}")
        with open(f"{base_path}_sim.json", "w") as f:
            json.dump(sim_output, f, indent=2)
        with open(f"{base_path}_real.json", "w") as f:
            json.dump(real_output, f, indent=2)

        print(f"\nSaved data files:")
        print(f"  {base_path}_sim.json")
        print(f"  {base_path}_real.json")