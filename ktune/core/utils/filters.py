import numpy as np


def detect_and_filter_spikes(position, velocity, time, pos_threshold_deg=50, vel_threshold_degs=200, window_size=3):
    """
    Detect and filter sudden spikes in pendulum position and velocity data.
    
    Args:
        position (np.array): Position measurements in degrees
        velocity (np.array): Velocity measurements in deg/s
        time (np.array): Time stamps
        pos_threshold_deg (float): Maximum allowed position change between consecutive samples
        vel_threshold_degs (float): Maximum allowed velocity change between consecutive samples
        window_size (int): Size of the window for median filtering
    
    Returns:
        tuple: (filtered_position, filtered_velocity, spike_indices, spike_report)
    """
    # Calculate differences between consecutive samples
    pos_diff = np.abs(np.diff(position))
    vel_diff = np.abs(np.diff(velocity))
    
    # Find indices where either position or velocity difference exceeds threshold
    pos_spike_indices = np.where(pos_diff > pos_threshold_deg)[0]
    vel_spike_indices = np.where(vel_diff > vel_threshold_degs)[0]
    spike_indices = np.unique(np.concatenate([pos_spike_indices, vel_spike_indices]))
    
    if len(spike_indices) > 0:
        print(f"\nDetected {len(spike_indices)} spikes in data:")
        for idx in spike_indices:
            print(f"  Time: {time[idx]:.3f}s")
            print(f"    Position change: {pos_diff[idx]:.1f} degrees")
            print(f"    Velocity change: {vel_diff[idx]:.1f} deg/s")
    
    # Filter spikes using median filter around detected points
    filtered_position = position.copy()
    filtered_velocity = velocity.copy()
    
    for idx in spike_indices:
        start_idx = max(0, idx - window_size//2)
        end_idx = min(len(position), idx + window_size//2 + 1)
        
        # Filter position
        valid_pos_points = [p for i, p in enumerate(position[start_idx:end_idx]) 
                          if abs(p - position[idx]) < pos_threshold_deg]
        if valid_pos_points:
            filtered_position[idx+1] = np.median(valid_pos_points)
        
        # Filter velocity
        valid_vel_points = [v for i, v in enumerate(velocity[start_idx:end_idx]) 
                          if abs(v - velocity[idx]) < vel_threshold_degs]
        if valid_vel_points:
            filtered_velocity[idx+1] = np.median(valid_vel_points)
    
    return filtered_position, filtered_velocity