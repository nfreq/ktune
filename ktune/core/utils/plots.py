import matplotlib.pyplot as plt
import numpy as np
from ktune import __version__
from ktune.core.utils import metrics
import os

class Plot:
    """Handles plotting of test results."""

    def __init__(self, config, sim_data, real_data):
        """Initialize the TestPlotter.
        
        Args:
            config: Test configuration object
            sim_data (dict): Simulation data
            real_data (dict): Real robot data
        """
        self.config = config
        self.sim_data = sim_data
        self.real_data = real_data

    def create_plots(self, timestamp: str, plot_dir: str):
        """Create and save all test plots.
        
        Args:
            timestamp (str): Timestamp for file naming
            plot_dir (str): Directory to save plots
        """
        self._create_time_history_plots(timestamp, plot_dir)
        if self.config.test == "chirp" and "freq_response" in self.sim_data:
            self._create_bode_plots(timestamp, plot_dir)

    def _create_time_history_plots(self, timestamp: str, plot_dir: str):
        """Create time history plots."""
        # Create figure and subplots
        fig, axs = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
        fig.suptitle(self._get_title_string(), fontsize=16)

        # Plot position data (top row)
        self._plot_position_data(axs[0, 0], self.sim_data, "Sim", "blue", "o-")
        self._plot_position_data(axs[0, 1], self.real_data, "Real", "red", "s-")

        # Plot velocity data (bottom row)
        self._plot_velocity_data(axs[1, 0], axs[1, 1])

        # Common formatting
        self._format_velocity_subplots(axs[1, 0], axs[1, 1])

        # Add version text and save
        plt.figtext(0.5, 0.02, f"ktune v{__version__}", ha='center', va='center', fontsize=12)
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.savefig(os.path.join(plot_dir, f"{timestamp}_{self.config.test}_time.png"))
        plt.close()

    def _create_bode_plots(self, timestamp: str, plot_dir: str):
        """Create Bode plots for chirp test results."""
        if "freq_response" not in self.sim_data or "freq_response" not in self.real_data:
            return

        fig_bode, (ax_mag, ax_phase) = plt.subplots(2, 1, figsize=(10, 10))
        fig_bode.suptitle(f"{self.config.name} - Frequency Response", fontsize=16)

        # Magnitude plot (dB)
        for data, label, color in [(self.sim_data, 'Sim', 'b'), 
                                (self.real_data, 'Real', 'r')]:
            freq = data["freq_response"]["freq"]
            mag = data["freq_response"]["magnitude"]
            phase = data["freq_response"]["phase"]
            coherence = data["freq_response"].get("coherence")

            # Plot magnitude - use '-' for line style separately
            ax_mag.semilogx(freq, 20 * np.log10(mag), '-', color=color, label=label)
            
            # Optionally plot coherence as transparency
            if coherence is not None:
                ax_mag.fill_between(freq, 20 * np.log10(mag), alpha=0.2, color=color)

        ax_mag.set_ylabel("Magnitude (dB)")
        ax_mag.set_title("Bode Plot")
        ax_mag.grid(True, which="both")
        ax_mag.legend()

        # Phase plot
        for data, label, color in [(self.sim_data, 'Sim', 'b'), 
                                (self.real_data, 'Real', 'r')]:
            freq = data["freq_response"]["freq"]
            phase = data["freq_response"]["phase"]
            ax_phase.semilogx(freq, phase, '-', color=color, label=label)

        ax_phase.set_xlabel("Frequency (Hz)")
        ax_phase.set_ylabel("Phase (degrees)")
        ax_phase.grid(True, which="both")
        ax_phase.legend()

        # Add bandwidth annotations if available
        for data, label, color in [(self.sim_data, 'Sim', 'b'), 
                                (self.real_data, 'Real', 'r')]:
            freq = data["freq_response"]["freq"]
            mag = data["freq_response"]["magnitude"]
            bandwidth = metrics.compute_bandwidth(freq, mag)
            if bandwidth:
                ax_mag.axvline(x=bandwidth, color=color, linestyle='--', alpha=0.5)
                ax_mag.text(bandwidth, -3, f'{label} BW: {bandwidth:.1f}Hz', 
                        rotation=90, verticalalignment='bottom')

        plt.tight_layout()
        plt.savefig(os.path.join(plot_dir, f"{timestamp}_{self.config.test}_bode.png"))
        plt.close()

    def _plot_position_data(self, ax, data, title_prefix, color, marker_style):
        """Plot position data on given axis."""
        ax.plot(data["cmd_time"], data["cmd_pos"], '--',
               color='black', linewidth=1, label='Command')
        ax.plot(data["time"], data["position"], marker_style,
               color=color, markersize=2, label='Actual')
        ax.set_title(f"{title_prefix} - Position")
        ax.set_ylabel("Position (deg)")
        ax.legend()
        ax.grid(True)

    def _plot_velocity_data(self, ax_sim, ax_real):
        """Plot velocity data based on test type."""
        if self.config.test in ["sine", "chirp"]:
            # Show commanded and actual velocities
            self._plot_full_velocity(ax_sim, self.sim_data, "blue", "o-")
            self._plot_full_velocity(ax_real, self.real_data, "red", "s-")
        elif self.config.test == "step":
            # Show only actual velocities
            self._plot_actual_velocity(ax_sim, self.sim_data, "blue", "o-")
            self._plot_actual_velocity(ax_real, self.real_data, "red", "s-")

    def _plot_full_velocity(self, ax, data, color, marker_style):
        """Plot commanded and actual velocity."""
        ax.plot(data["cmd_time"], data["cmd_vel"], '--',
               color='black', linewidth=1, label='Command')
        ax.plot(data["time"], data["velocity"], marker_style,
               color=color, markersize=2, label='Actual')

    def _plot_actual_velocity(self, ax, data, color, marker_style):
        """Plot only actual velocity."""
        ax.plot(data["time"], data["velocity"], marker_style,
               color=color, markersize=2, label='Actual')

    def _format_velocity_subplots(self, ax_sim, ax_real):
        """Apply common formatting to velocity subplots."""
        ax_sim.set_title("Sim - Velocity")
        ax_sim.set_xlabel("Time (s)")
        ax_sim.set_ylabel("Velocity (deg/s)")
        ax_sim.legend()
        ax_sim.grid(True)

        ax_real.set_title("Real - Velocity")
        ax_real.set_xlabel("Time (s)")
        ax_real.legend()
        ax_real.grid(True)

    def _get_title_string(self):
        """Generate plot title based on test type."""
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
        joint_name = JOINT_NAMES.get(self.config.actuator_id, f"id_{self.config.actuator_id}")

        if self.config.test == "chirp":
            return (
                f"{self.config.name} -- Chirp Test -- ID: {self.config.actuator_id} {joint_name}\n"
                f"Center: {self.config.start_pos}°, Init Freq: {self.config.chirp_init_freq} Hz, "
                f"Sweep Rate: {self.config.chirp_sweep_rate} Hz/s, Amp: {self.config.chirp_amp}°, "
                f"Duration: {self.config.chirp_duration}s\n"
                f"Update Rate: {self.config.sample_rate} Hz\n"
                f"Sim Kp: {self.config.sim_kp} Kv: {self.config.sim_kv} | "
                f"Real Kp: {self.config.kp} Kd: {self.config.kd} Ki: {self.config.ki}\n"
                f"Acceleration: {self.config.acceleration:.0f} deg/s²"
            )
        elif self.config.test == "sine":
            return (
                f"{self.config.name} -- Sine Wave Test -- ID: {self.config.actuator_id} {joint_name}\n"
                f"Center: {self.config.start_pos}°, Freq: {self.config.freq} Hz, "
                f"Amp: {self.config.amp}°, Sample/Ctrl Rate: {self.config.sample_rate} Hz\n"
                f"Sim Kp: {self.config.sim_kp} Kv: {self.config.sim_kv} | "
                f"Real Kp: {self.config.kp} Kd: {self.config.kd} Ki: {self.config.ki}\n"
                f"Acceleration: {self.config.acceleration:.0f} deg/s²"
            )
        elif self.config.test == "step":
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
            # Calculate average metrics across all steps
            sim_avg = {
                'overshoot': np.mean([m['overshoot'] for m in sim_metrics]),
                'rise_time': np.mean([m['rise_time'] for m in sim_metrics if m['rise_time'] is not None]),
                'settling_time': np.mean([m['settling_time'] for m in sim_metrics if m['settling_time'] is not None])
            }
            real_avg = {
                'overshoot': np.mean([m['overshoot'] for m in real_metrics]),
                'rise_time': np.mean([m['rise_time'] for m in real_metrics if m['rise_time'] is not None]),
                'settling_time': np.mean([m['settling_time'] for m in real_metrics if m['settling_time'] is not None])
            }

            return (
                f"{self.config.name} -- Step Test -- ID: {self.config.actuator_id} {joint_name}\n"
                f"Center: {self.config.start_pos}°, Step Size: ±{self.config.step_size}°, "
                f"Hold: {self.config.step_hold_time}s, Count: {self.config.step_count}\n"
                f"Update Rate: {self.config.sample_rate} Hz\n"
                f"Sim Kp: {self.config.sim_kp} Kv: {self.config.sim_kv} | "
                f"Real Kp: {self.config.kp} Kd: {self.config.kd} Ki: {self.config.ki}\n"
                f"Sim Metrics - Overshoot: {sim_avg['overshoot']:.1f}% Rise: {sim_avg['rise_time']:.3f}s Settling: {sim_avg['settling_time']:.3f}s\n"
                f"Real Metrics - Overshoot: {real_avg['overshoot']:.1f}% Rise: {real_avg['rise_time']:.3f}s Settling: {real_avg['settling_time']:.3f}s\n"
                f"Acceleration: {self.config.acceleration:.0f} deg/s²"
            )
        else:
            return f"{self.config.test.capitalize()} Test - Actuator {self.config.actuator_id}"