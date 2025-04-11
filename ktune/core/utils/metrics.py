import numpy as np
from scipy.signal import coherence, csd
from scipy.interpolate import interp1d
from pathlib import Path
import json
from typing import Dict

# Calculate tracking metrics
def compute_tracking_error(cmd_time, cmd_pos, actual_time, actual_pos):
    """Compute RMS tracking error"""
    # Interpolate commanded positions to actual timestamps
    from scipy.interpolate import interp1d
    cmd_interp = interp1d(cmd_time, cmd_pos, bounds_error=False)
    cmd_at_actual = cmd_interp(actual_time)
    
    # Compute RMS error where we have both commanded and actual
    valid_idx = ~np.isnan(cmd_at_actual)
    if not np.any(valid_idx):
        return float('nan')
    
    errors = cmd_at_actual[valid_idx] - np.array(actual_pos)[valid_idx]
    rms_error = np.sqrt(np.mean(np.square(errors)))
    return rms_error

def compute_tracking_metrics(cmd_time, cmd_pos, actual_time, actual_pos, cmd_vel=None, actual_vel=None):
    """Compute tracking metrics between commanded and actual values.
    
    Args:
        cmd_time (array-like): Command timestamps
        cmd_pos (array-like): Commanded positions
        actual_time (array-like): Actual timestamps
        actual_pos (array-like): Actual positions
        cmd_vel (array-like, optional): Commanded velocities
        actual_vel (array-like, optional): Actual velocities
    
    Returns:
        dict: Dictionary containing position and velocity metrics
    """
    # Interpolate actual positions to command timestamps
    actual_interp = interp1d(actual_time, actual_pos, bounds_error=False, fill_value=np.nan)
    actual_resampled = actual_interp(cmd_time)
    
    # Remove NaN values
    valid = ~np.isnan(actual_resampled)
    if not np.any(valid):
        return {}
        
    t = np.array(cmd_time)[valid]
    cmd = np.array(cmd_pos)[valid]
    act = actual_resampled[valid]

    # Calculate position errors
    errors = cmd - act
    abs_errors = np.abs(errors)
    
    metrics = {
        "position": {
            "rms_error": float(np.sqrt(np.mean(np.square(errors)))),
            "max_error": float(np.max(abs_errors)),
            "mean_error": float(np.mean(errors)),
            "mean_abs_error": float(np.mean(abs_errors)),
            "std_error": float(np.std(errors))
        }
    }

    # Add velocity metrics if available
    if cmd_vel is not None and actual_vel is not None:
        vel_interp = interp1d(actual_time, actual_vel, bounds_error=False, fill_value=np.nan)
        vel_resampled = vel_interp(cmd_time)
        valid_vel = ~np.isnan(vel_resampled)
        
        if np.any(valid_vel):
            vel_errors = np.array(cmd_vel)[valid_vel] - vel_resampled[valid_vel]
            abs_vel_errors = np.abs(vel_errors)
            
            metrics["velocity"] = {
                "rms_error": float(np.sqrt(np.mean(np.square(vel_errors)))),
                "max_error": float(np.max(abs_vel_errors)),
                "mean_error": float(np.mean(vel_errors)),
                "mean_abs_error": float(np.mean(abs_vel_errors)),
                "std_error": float(np.std(vel_errors))
            }

    return metrics

def compute_data_statistics(time, position, velocity):
    """Compute basic statistics for position and velocity data.
    
    Args:
        time (array-like): Timestamps
        position (array-like): Position values
        velocity (array-like): Velocity values
    
    Returns:
        dict: Dictionary containing position and velocity statistics
    """
    return {
        "position": {
            "min": float(np.min(position)),
            "max": float(np.max(position)),
            "mean": float(np.mean(position)),
            "std": float(np.std(position))
        },
        "velocity": {
            "min": float(np.min(velocity)),
            "max": float(np.max(velocity)),
            "mean": float(np.mean(velocity)),
            "std": float(np.std(velocity))
        },
        "sample_count": len(time),
        "actual_sample_rate": float(1.0 / np.mean(np.diff(time)))
    }

def compute_frequency_response(cmd_time, cmd_pos, actual_time, actual_pos):
    """Compute frequency response metrics including magnitude and phase."""
    # Debug input data
    print(f"\nFrequency Response Input:")
    print(f"Command data points: {len(cmd_time)}")
    print(f"Actual data points: {len(actual_time)}")
    
    # Convert inputs to numpy arrays
    cmd_time = np.array(cmd_time)
    cmd_pos = np.array(cmd_pos)
    actual_time = np.array(actual_time)
    actual_pos = np.array(actual_pos)
    
    # Debug time ranges
    print(f"Command time range: {cmd_time[0]:.3f} to {cmd_time[-1]:.3f}")
    print(f"Actual time range: {actual_time[0]:.3f} to {actual_time[-1]:.3f}")
    print(f"Command position range: {np.min(cmd_pos):.3f} to {np.max(cmd_pos):.3f}")
    print(f"Actual position range: {np.min(actual_pos):.3f} to {np.max(actual_pos):.3f}")
    
    # Ensure time arrays are strictly increasing
    if not np.all(np.diff(actual_time) > 0):
        print("Warning: Actual time array is not strictly increasing")
        # Sort the arrays by time
        sort_idx = np.argsort(actual_time)
        actual_time = actual_time[sort_idx]
        actual_pos = actual_pos[sort_idx]
    
    # Interpolate actual positions to command timestamps for analysis
    try:
        actual_interp = interp1d(actual_time, actual_pos, bounds_error=False, fill_value=np.nan)
        actual_resampled = actual_interp(cmd_time)
        
        # Remove NaN values
        valid = ~np.isnan(actual_resampled)
        print(f"Valid interpolation points: {np.sum(valid)} out of {len(cmd_time)}")
        
        if not np.any(valid):
            print("No valid data points after interpolation")
            return {
                "freq": [],
                "magnitude": [],
                "phase": [],
                "coherence": []
            }
                
        t = cmd_time[valid]
        cmd = cmd_pos[valid]
        act = actual_resampled[valid]

        # Compute frequency response
        fs = 1.0 / np.mean(np.diff(t))  # Average sample rate
        print(f"Computed sample rate: {fs:.1f} Hz")
        
        f, coh = coherence(cmd, act, fs=fs)
        f, Pxx = csd(cmd, act, fs=fs)
        
        # Calculate magnitude and phase
        mag = np.abs(Pxx)
        phase = np.angle(Pxx, deg=True)
        
        print(f"Computed {len(f)} frequency points")
        return {
            "freq": f.tolist(),
            "magnitude": mag.tolist(),
            "phase": phase.tolist(),
            "coherence": coh.tolist()
        }
    except Exception as e:
        print(f"Error in frequency response computation: {str(e)}")
        import traceback
        traceback.print_exc()
        return {
            "freq": [],
            "magnitude": [],
            "phase": [],
            "coherence": []
        }

def analyze_frequency_response(sim_data=None, real_data=None):
    """Analyze frequency response for both simulation and real system."""
    results = {}
    
    if sim_data is not None:
        print("\nAnalyzing Simulation Data:")
        results["sim"] = compute_frequency_response(
            sim_data["cmd_time"],
            sim_data["cmd_pos"],
            sim_data["time"],
            sim_data["position"]
        )
    
    if real_data is not None:
        print("\nAnalyzing Real System Data:")
        results["real"] = compute_frequency_response(
            real_data["cmd_time"],
            real_data["cmd_pos"],
            real_data["time"],
            real_data["position"]
        )
    return results

def compute_bandwidth(freq, magnitude):
    """Compute the -3dB bandwidth from frequency response data."""
    if not freq or not magnitude or len(freq) == 0 or len(magnitude) == 0:
        print("Warning: Empty frequency response data")
        return None

    # Convert magnitude to dB
    mag_db = 20 * np.log10(np.abs(magnitude))
    
    # Normalize to 0dB at DC
    mag_db = mag_db - mag_db[0]
    
    # Debug prints
    print(f"Frequency points: {len(freq)}")
    print(f"Magnitude range: {mag_db.min():.1f}dB to {mag_db.max():.1f}dB")
    
    # Find first crossing of -3dB
    cutoff_idx = np.where(mag_db <= -3)[0]
    
    if len(cutoff_idx) > 0:
        bandwidth = float(freq[cutoff_idx[0]])
        print(f"Found -3dB point at {bandwidth:.1f}Hz")
        return bandwidth
    else:
        print("No -3dB crossing found in data")
        return None
    
def compute_step_metrics(time_array, pos_array, step_size, hold_time, step_count):
    """
    Compute comprehensive step response metrics.
    
    Args:
        time_array: Array of time stamps (seconds)
        pos_array: Array of measured positions (degrees)
        step_size: Size of each step (degrees)
        hold_time: Duration to hold each position (seconds)
        step_count: Number of steps to analyze
    """
    time_array = np.array(time_array)
    pos_array = np.array(pos_array)
    start_pos = pos_array[0]

    # Build step sequence
    steps = [(start_pos, hold_time)]
    for _ in range(step_count):
        steps.append((start_pos + step_size, hold_time))
        steps.append((start_pos, hold_time))

    # Build array of step command times
    step_times = [0.0]
    for (target, duration) in steps:
        step_times.append(step_times[-1] + duration)

    step_metrics = []
    print("\nStep Response Metrics:")
    
    for i in range(1, len(steps)):
        old_target = steps[i-1][0]
        new_target = steps[i][0]
        command_time = step_times[i]
        window_end = step_times[i] + hold_time

        # Find data within step window
        idx = np.where((time_array >= command_time) & 
                      (time_array <= window_end))[0]
        if len(idx) == 0:
            continue
            
        t_window = time_array[idx] - command_time
        p_window = pos_array[idx]
        
        # Calculate step metrics
        metrics = {}
        
         # Overshoot calculation
        step_size_actual = abs(step_size)  # Use the commanded step size
        if step_size_actual > 0:
            if step_size > 0:  # Positive step (moving up)
                # If we go higher than target, that's overshoot
                overshoot = max(0.0, (p_window.max() - new_target) / step_size_actual * 100.0)
            else:  # Negative step (moving down)
                # If we go lower than target, that's overshoot
                overshoot = max(0.0, (new_target - p_window.min()) / step_size_actual * 100.0)
        else:
            overshoot = 0.0
            
        metrics['overshoot'] = overshoot

    
        # Rise time (10% to 90%)
        ten_pct = old_target + 0.1 * (new_target - old_target)
        ninety_pct = old_target + 0.9 * (new_target - old_target)
        
        # Find crossing times
        t_10 = t_window[np.where(np.diff(np.signbit(p_window - ten_pct)))[0]]
        t_90 = t_window[np.where(np.diff(np.signbit(p_window - ninety_pct)))[0]]
        
        if len(t_10) > 0 and len(t_90) > 0:
            metrics['rise_time'] = float(t_90[0] - t_10[0])
        else:
            metrics['rise_time'] = None

        # Settling time (±2% of step size)
        settling_band = step_size_actual * 0.02
        settled_idx = np.where(abs(p_window - new_target) <= settling_band)[0]
        if len(settled_idx) > 0:
            metrics['settling_time'] = float(t_window[settled_idx[0]])
        else:
            metrics['settling_time'] = None

        # Peak time
        peak_idx = np.argmax(abs(p_window - new_target))
        metrics['peak_time'] = float(t_window[peak_idx])
        
        # Debug prints
        print(f"\nStep {i}:")
        print(f"  Old target: {old_target:.2f}°")
        print(f"  New target: {new_target:.2f}°")
        print(f"  Step size: {step_size_actual:.2f}°")
        print(f"  Overshoot: {metrics['overshoot']:.1f}%")
        print(f"  Rise time: {metrics['rise_time']:.3f}s" if metrics['rise_time'] else "  Rise time: N/A")
        print(f"  Settling time: {metrics['settling_time']:.3f}s" if metrics['settling_time'] else "  Settling time: N/A")
        print(f"  Peak time: {metrics['peak_time']:.3f}s")
        
        step_metrics.append(metrics)

    return step_metrics

def analyze_sysid_data(data: Dict) -> Dict:
    """Analyze system identification data quality
    
    Args:
        data: Dictionary containing experiment data and config
    """
    # Extract timestamps and convert to numpy array
    timestamps = np.array([entry['timestamp'] for entry in data['entries']])
    dt = np.diff(timestamps)
    
    metrics = {
        # Sampling statistics
        'total_samples': len(timestamps),
        'duration': timestamps[-1] - timestamps[0],
        'dt_mean': np.mean(dt),
        'dt_std': np.std(dt),
        'dt_min': np.min(dt),
        'dt_max': np.max(dt),
        'actual_rate': 1.0/np.mean(dt),
        'target_rate': data['sample_rate'],
        
        # Data completeness
        'missing_samples': sum(dt > (2.0 * np.mean(dt))),  # Gaps > 2x mean dt
        
        # Trajectory info
        'trajectory_type': data['trajectory'],
    }
    
    return metrics
    
    return metrics