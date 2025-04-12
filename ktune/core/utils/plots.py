import matplotlib.pyplot as plt
import numpy as np
from ktune import __version__
from ktune.core.utils import metrics
import os
from pathlib import Path
class Plot:
    """Handles plotting of test results."""

    def __init__(self, config, sim_data=None, real_data=None):
        """Initialize the TestPlotter.
        
        Args:
            config: Test configuration object
            sim_data (dict, optional): Simulation data
            real_data (dict, optional): Real robot data
        """
        self.config = config
        self.mode = config.mode
        self.sim_data = sim_data
        self.real_data = real_data
        print(f"**********sim_data keys: {self.sim_data.keys()}")

    def create_plots(self, timestamp: str, plot_dir: str):
        """Create and save all test plots.
        
        Args:
            timestamp (str): Timestamp for file naming
            plot_dir (str): Directory to save plots
        """
        self._create_time_history_plots(timestamp, plot_dir)
        if (self.config.test == "chirp" and 
            ((self.sim_data and "freq_response" in self.sim_data) or
             (self.real_data and "freq_response" in self.real_data))):
            self._create_bode_plots(timestamp, plot_dir)

    def _create_time_history_plots(self, timestamp: str, plot_dir: str):
        """Create time history plots with overlaid real and sim data."""
        # Create figure with two subplots (position and velocity)
        fig, (ax_pos, ax_vel) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
        fig.suptitle(self._get_title_string(), fontsize=16)

        # Plot data based on mode
        if self.mode in ['compare', 'sim']:
            # Plot simulation data in blue
            ax_pos.plot(self.sim_data["cmd_time"], self.sim_data["cmd_pos"], 'k--',
                    linewidth=1, label='Command')
            ax_pos.plot(self.sim_data["time"], self.sim_data["position"], 'b-',
                    linewidth=1, label='Sim')
            
            if self.config.test in ["sine", "chirp"]:
                ax_vel.plot(self.sim_data["cmd_time"], self.sim_data["cmd_vel"], 'k--',
                        linewidth=1, label='Command')
            ax_vel.plot(self.sim_data["time"], self.sim_data["velocity"], 'b-',
                    linewidth=1, label='Sim')

        if self.mode in ['compare', 'real']:
            # Plot real data in red
            if self.mode != 'compare':  # Only plot command if not already plotted
                ax_pos.plot(self.real_data["cmd_time"], self.real_data["cmd_pos"], 'k--',
                        linewidth=1, label='Command')
                if self.config.test in ["sine", "chirp"]:
                    ax_vel.plot(self.real_data["cmd_time"], self.real_data["cmd_vel"], 'k--',
                            linewidth=1, label='Command')
            
            ax_pos.plot(self.real_data["time"], self.real_data["position"], 'r-',
                    linewidth=1, label='Real')
            ax_vel.plot(self.real_data["time"], self.real_data["velocity"], 'r-',
                    linewidth=1, label='Real')

        # Format position plot
        ax_pos.set_title("Position")
        ax_pos.set_ylabel("Position (deg)")
        ax_pos.grid(True)
        ax_pos.legend()

        # Format velocity plot
        ax_vel.set_title("Velocity")
        ax_vel.set_xlabel("Time (s)")
        ax_vel.set_ylabel("Velocity (deg/s)")
        ax_vel.grid(True)
        ax_vel.legend()

        # Add version text and save
        plt.figtext(0.5, 0.02, f"ktune v{__version__}", ha='center', va='center', fontsize=12)
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.savefig(os.path.join(plot_dir, f"{timestamp}_{self.config.test}_time.png"))
        plt.close()

    def _plot_position_data(self, ax, data, title_prefix, color, marker_style):
        """Plot position data on given axis."""
        if data is None:
            return
        ax.plot(data["cmd_time"], data["cmd_pos"], '--',
               color='black', linewidth=1, label='Command')
        ax.plot(data["time"], data["position"], marker_style,
               color=color, markersize=2, label='Actual')
        ax.set_title(f"{title_prefix} - Position")
        ax.set_ylabel("Position (deg)")
        ax.legend()
        ax.grid(True)

    def _plot_velocity_data_single(self, ax, data, title_prefix, color, marker_style):
        """Plot velocity data for a single system."""
        if data is None:
            return
            
        if self.config.test in ["sine", "chirp"]:
            # Show commanded and actual velocities
            ax.plot(data["cmd_time"], data["cmd_vel"], '--',
                   color='black', linewidth=1, label='Command')
            ax.plot(data["time"], data["velocity"], marker_style,
                   color=color, markersize=2, label='Actual')
        else:  # step test
            # Show only actual velocities
            ax.plot(data["time"], data["velocity"], marker_style,
                   color=color, markersize=2, label='Actual')

        ax.set_title(f"{title_prefix} - Velocity")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Velocity (deg/s)")
        ax.legend()
        ax.grid(True)

    def _create_bode_plots(self, timestamp: str, plot_dir: str):
        """Create Bode plots for chirp test results."""
        try:
            fig_bode, (ax_mag, ax_phase) = plt.subplots(2, 1, figsize=(10, 10))
            fig_bode.suptitle(f"{self.config.name} - Frequency Response", fontsize=16)

            # Define available datasets
            datasets = {
                'Sim': (self.sim_data, 'b') if self.mode in ['compare', 'sim'] else None,
                'Real': (self.real_data, 'r') if self.mode in ['compare', 'real'] else None
            }
            
            valid_data = False  # Track if we have any valid data to plot
            
            # Filter out None values and plot available data
            for label, (data, color) in {k: v for k, v in datasets.items() if v is not None}.items():
                if not data or "freq_response" not in data:
                    print(f"Warning: No frequency response data for {label}")
                    continue

                freq_response = data["freq_response"]
                # Validate required frequency response data
                if not all(key in freq_response for key in ["freq", "magnitude", "phase"]):
                    print(f"Warning: Incomplete frequency response data for {label}")
                    continue

                valid_data = True
                freq = freq_response["freq"]
                mag = freq_response["magnitude"]
                phase = freq_response["phase"]
                coherence = freq_response.get("coherence")

                # Plot magnitude
                ax_mag.semilogx(freq, 20 * np.log10(mag), '-', color=color, label=label)
                if coherence is not None:
                    ax_mag.fill_between(freq, 20 * np.log10(mag), alpha=0.2, color=color)

                # Plot phase
                ax_phase.semilogx(freq, phase, '-', color=color, label=label)

                # Add bandwidth annotation if we can compute it
                try:
                    bandwidth = metrics.compute_bandwidth(freq, mag)
                    if bandwidth:
                        ax_mag.axvline(x=bandwidth, color=color, linestyle='--', alpha=0.5)
                        ax_mag.text(bandwidth, -3, f'{label} BW: {bandwidth:.1f}Hz', 
                                rotation=90, verticalalignment='bottom')
                except Exception as e:
                    print(f"Warning: Could not compute bandwidth for {label}: {e}")

            if valid_data:
                # Format plots
                ax_mag.set_ylabel("Magnitude (dB)")
                ax_mag.set_title("Bode Plot")
                ax_mag.grid(True, which="both")
                ax_mag.legend()

                ax_phase.set_xlabel("Frequency (Hz)")
                ax_phase.set_ylabel("Phase (degrees)")
                ax_phase.grid(True, which="both")
                ax_phase.legend()

                plt.tight_layout()
                plt.savefig(os.path.join(plot_dir, f"{timestamp}_{self.config.test}_bode.png"))
            else:
                print("Warning: No valid frequency response data available for Bode plots")
                
            plt.close()

        except Exception as e:
            print(f"Warning: Could not create Bode plots: {e}")
            plt.close()  # Ensure figure is closed even if there's an error



    def _get_title_string(self):
        """Generate plot title based on test type."""
        JOINT_NAMES = { # these don't belong here.
            #11: "Left Shoulder Roll", 12: "Left Shoulder Pitch",
            #13: "Left Elbow Roll", 14: "Left Gripper",
            #21: "Right Shoulder Roll", 22: "Right Shoulder Pitch",
            #23: "Right Elbow Roll", 24: "Right Gripper",
            #31: "Left Hip Yaw", 32: "Left Hip Roll",
            #33: "Left Hip Pitch", 34: "Left Knee Pitch",
            #35: "Left Ankle Pitch", 41: "Right Hip Yaw",
            #42: "Right Hip Roll", 43: "Right Hip Pitch",
            #44: "Right Knee Pitch", 45: "Right Ankle Pitch"
        }
        joint_name = JOINT_NAMES.get(self.config.actuator_id, f"id_{self.config.actuator_id}")

        base_info = f"{self.config.name} -- {self.config.test.capitalize()} Test -- ID: {self.config.actuator_id}\n" #{joint_name}\n"
        accel_string = f"Acceleration: {self.config.acceleration:.0f} deg/s²\n"

        if self.config.test == "chirp":
            return (
                f"{base_info}"
                f"Center: {self.config.start_pos}°, Init Freq: {self.config.chirp_init_freq} Hz, "
                f"Sweep Rate: {self.config.chirp_sweep_rate} Hz/s, Amp: {self.config.chirp_amp}°, "
                f"Duration: {self.config.chirp_duration}s\n"
                f"{self._get_gains_string()}\n"
                f"{accel_string}"
            )
        elif self.config.test == "sine":
            return (
                f"{base_info}"
                f"Center: {self.config.start_pos}°, Freq: {self.config.freq} Hz, "
                f"Amp: {self.config.amp}°, Sample/Ctrl Rate: {self.config.sample_rate} Hz\n"
                f"{self._get_gains_string()}\n"
                f"{accel_string}"
            )
        elif self.config.test == "step":
            metrics_string = self._get_step_metrics_string()
            return (
                f"{base_info}"
                f"Center: {self.config.start_pos}°, Step Size: ±{self.config.step_size}°, "
                f"Hold: {self.config.step_hold_time}s, Count: {self.config.step_count}\n"
                f"Update Rate: {self.config.sample_rate} Hz\n"
                f"{self._get_gains_string()}\n"
                #f"{metrics_string}"
                f"{accel_string}"
            )
        else:
            return f"{self.config.test.capitalize()} Test - Actuator {self.config.actuator_id}"
        
        
    def _get_gains_string(self):
        """Generate gains string based on mode."""
        gains = []
        if self.mode in ['compare', 'sim']:
            gains.append(f"Sim Kp: {self.config.sim_kp} Kd: {self.config.sim_kd}")
        if self.mode in ['compare', 'real']:
            gains.append(f"Real Kp: {self.config.kp} Kd: {self.config.kd} Ki: {self.config.ki}")
        return " | ".join(gains)

    def _get_step_metrics_string(self):
        """Generate step metrics string based on mode."""
        metrics_strings = []
        
        if self.mode in ['compare', 'sim'] and self.sim_data:
            sim_metrics = metrics.compute_step_metrics(
                np.array(self.sim_data["time"]),
                np.array(self.sim_data["position"]),
                self.config.step_size,
                self.config.step_hold_time,
                self.config.step_count
            )
            if sim_metrics:
                sim_avg = {
                    'overshoot': np.mean([m['overshoot'] for m in sim_metrics]),
                    'rise_time': np.mean([m['rise_time'] for m in sim_metrics if m['rise_time'] is not None]),
                    'settling_time': np.mean([m['settling_time'] for m in sim_metrics if m['settling_time'] is not None])
                }
                metrics_strings.append(
                    f"Sim Metrics - Overshoot: {sim_avg['overshoot']:.1f}% "
                    f"Rise: {sim_avg['rise_time']:.3f}s "
                    f"Settling: {sim_avg['settling_time']:.3f}s"
                )

        if self.mode in ['compare', 'real'] and self.real_data:
            real_metrics = metrics.compute_step_metrics(
                np.array(self.real_data["time"]),
                np.array(self.real_data["position"]),
                self.config.step_size,
                self.config.step_hold_time,
                self.config.step_count
            )
            if real_metrics:
                real_avg = {
                    'overshoot': np.mean([m['overshoot'] for m in real_metrics]),
                    'rise_time': np.mean([m['rise_time'] for m in real_metrics if m['rise_time'] is not None]),
                    'settling_time': np.mean([m['settling_time'] for m in real_metrics if m['settling_time'] is not None])
                }
                metrics_strings.append(
                    f"Real Metrics - Overshoot: {real_avg['overshoot']:.1f}% "
                    f"Rise: {real_avg['rise_time']:.3f}s "
                    f"Settling: {real_avg['settling_time']:.3f}s"
                )

        return "\n".join(metrics_strings) + "\n" if metrics_strings else ""
        
class PendulumPlot:
    """Handles plotting of pendulum system identification results."""

    def __init__(self, data: dict):
        """Initialize with experiment data.
        
        Args:
            data: Dictionary containing experiment data and config
        """
        self.data = data
        self.entries = data['entries']

    def create_plots(self, save_dir: str | Path, timestamp: str = None):
        """Create and save analysis plots.
        
        Args:
            save_dir: Directory to save plots
            timestamp: Optional timestamp for file naming
        """
        # Extract time series data
        t = np.array([entry['timestamp'] for entry in self.entries])
        pos = np.array([entry['position'] for entry in self.entries])
        vel = np.array([entry['speed'] for entry in self.entries])
        torque = np.array([entry['torque'] for entry in self.entries])
        goal_pos = np.array([entry['goal_position'] for entry in self.entries])
        torque_enabled = np.array([entry['torque_enable'] for entry in self.entries])

        # Create main results plot
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        fig.suptitle(f"Pendulum System ID - {self.data['trajectory']}", fontsize=14)
        
        # Position tracking plot
        ax1.plot(t, np.rad2deg(goal_pos), 'k--', label='Command', linewidth=1)
        ax1.plot(t, np.rad2deg(pos), 'b-', label='Actual', linewidth=1)
        ax1.set_ylabel('Position (deg)')
        ax1.grid(True)
        ax1.legend()
        
        # Velocity plot
        ax2.plot(t, np.rad2deg(vel), 'g-', label='Velocity', linewidth=1)
        ax2.set_ylabel('Velocity (deg/s)')
        ax2.grid(True)
        ax2.legend()
        
        # Torque plot with enable status
        ax3.plot(t, torque, 'r-', label='Torque', linewidth=1)
        # Add shaded regions for torque disabled periods
        for i in range(len(t)-1):
            if not torque_enabled[i]:
                ax3.axvspan(t[i], t[i+1], color='gray', alpha=0.3)
        ax3.set_ylabel('Torque')
        ax3.set_xlabel('Time (s)')
        ax3.grid(True)
        ax3.legend()
        
        # Add configuration details
        config_text = (
            f"Mass: {self.data['mass']}kg, "
            f"Length: {self.data['length']}m\n"
            f"Control: Kp={self.data['kp']}, "
            f"Kd={self.data['kd']}, "
            f"Ki={self.data['ki']}, "
            f"Max Torque={self.data['max_torque']}"
        )
        fig.text(0.1, 0.01, config_text, fontsize=10)
        
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        
        # Save results plot
        save_path = Path(save_dir) / f"pendulum_sysid_{self.data['trajectory']}"
        if timestamp:
            save_path = save_path.with_name(f"{timestamp}_{save_path.name}")
        plt.savefig(f"{save_path}_results.png")
        plt.close()
        
        # Create error analysis plot
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
        fig.suptitle(f"Tracking Error Analysis - {self.data['trajectory']}", fontsize=14)
        
        # Position error
        pos_error = np.rad2deg(goal_pos - pos)
        ax1.plot(t, pos_error, 'b-', label='Position Error')
        ax1.set_ylabel('Position Error (deg)')
        ax1.grid(True)
        ax1.legend()
        
        # Error histogram
        ax2.hist(pos_error, bins=50, density=True)
        ax2.set_xlabel('Position Error (deg)')
        ax2.set_ylabel('Density')
        ax2.grid(True)
        
        # Add error statistics
        stats_text = (
            f"RMS Error: {np.sqrt(np.mean(pos_error**2)):.2f}°\n"
            f"Mean Error: {np.mean(pos_error):.2f}°\n"
            f"Max Error: {np.max(np.abs(pos_error)):.2f}°"
        )
        ax2.text(0.95, 0.95, stats_text, 
                transform=ax2.transAxes,
                verticalalignment='top',
                horizontalalignment='right',
                bbox=dict(facecolor='white', alpha=0.8))
        
        plt.tight_layout()
        plt.savefig(f"{save_path}_error.png")
        plt.close()