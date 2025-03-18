# Credit to https://github.com/Rhoban/bam !!

from .base import TestBench, TestConfig
from dataclasses import dataclass
import numpy as np
import asyncio
from typing import Dict
from datetime import datetime
from ktune.core.utils.plots import PendulumPlot
from ktune.core.utils.filters import detect_and_filter_spikes
from ktune.core.utils import metrics
from pathlib import Path

@dataclass
class PendulumConfig(TestConfig):
    """Configuration for pendulum experiments"""
    motor: str
    mass: float
    length: float
    vin: float = 12.0
    offset: float = 0.0  # Radians, offset from motor zero to pendulum bottom
    sample_rate: float = 100.0


class PendulumTrajectory:
    """Base class for pendulum trajectories"""

    def cubic_interpolate(self, keyframes: list, t: float):
        if t < keyframes[0][0]: return keyframes[0][1]
        if t > keyframes[-1][0]: return keyframes[-1][1]
        
        for i in range(len(keyframes) - 1):
            if keyframes[i][0] <= t <= keyframes[i + 1][0]:
                t0, x0, x0p = keyframes[i]
                t1, x1, x1p = keyframes[i + 1]
                
                A = [[1, t0, t0**2, t0**3],
                        [0, 1, 2*t0, 3*t0**2],
                        [1, t1, t1**2, t1**3],
                        [0, 1, 2*t1, 3*t1**2]]
                b = [x0, x0p, x1, x1p]
                w = np.linalg.solve(A, b)
                return w[0] + w[1]*t + w[2]*t**2 + w[3]*t**3

    def __call__(self, t: float):
        """Return (angle, torque_enable) at time t"""
        raise NotImplementedError

class LiftAndDrop(PendulumTrajectory):
    duration = 6.0
    def __call__(self, t: float):
        keyframes = [[0.0, 0.0, 0.0], [2.0, -np.pi/2, 0.0]]
        angle = self.cubic_interpolate(keyframes, t)
        enable = t < 2.0
        return angle, enable

class SinusTimeSquare(PendulumTrajectory):
    duration = 6.0
    def __call__(self, t: float):
        angle = np.sin(t**2)
        return angle, True

class Chirp(PendulumTrajectory):
    duration = 6.0
    def __call__(self, t: float):
        # Exponential chirp from 0.2 Hz to 0.8 Hz (staying under 2250°/s² limit)
        f0, f1 = 0.2, 0.8  # Hz - further reduced max frequency
        beta = np.log(f1/f0) / self.duration
        phase = 2*np.pi * f0 * (np.exp(beta * t) - 1) / beta
        angle = (np.pi/2) * np.sin(phase)
        return angle, True

class UpAndDown(PendulumTrajectory):
    duration = 6.0
    def __call__(self, t: float):
        keyframes = [
            [0.0, 0.0, 0.0],
            [3.0, np.pi/2, 0.0],
            [6.0, 0.8 * np.pi/2, 0.0],
        ]
        angle = self.cubic_interpolate(keyframes, t)
        return angle, True

class SinSin(PendulumTrajectory):
    duration = 6.0
    def __call__(self, t: float):
        # Base motion
        base_motion = np.sin(t)
        # Modulation (max amplitude of this term is 0.5)
        modulation = np.sin(5.0 * t) * 0.5 * np.sin(t * 2.0)
        # Combined motion
        raw_angle = base_motion + modulation

         # Scale factor so max angle is ~96 deg
        scale = 0.828  # from (96 deg) / (115.88 deg)

        # Scale to ±90 degrees (±π/2 radians)
        angle = raw_angle * (np.pi/2) * scale #(prevent disc mass from hitting testbench frame)
        return angle, True

class Brutal(PendulumTrajectory):
    duration = 6.0
    def __call__(self, t: float):
        if t > self.duration/4 and t < self.duration/1.5:
            return np.pi/2, True
        return 0.0, True

class Nothing(PendulumTrajectory):
    duration = 3.0
    def __call__(self, t: float):
        return 0.0, False
    

class PendulumBench(TestBench):
    """Pendulum testbed implementation"""
    
    def __init__(self, config: PendulumConfig):
        super().__init__(config)
        self.config: PendulumConfig = config
        self.trajectories = {
            "lift_and_drop": LiftAndDrop(),
            "sin_time_square": SinusTimeSquare(),
            "up_and_down": UpAndDown(),
            "sin_sin": SinSin(),
            "brutal": Brutal(),
            "nothing": Nothing(),
            "chirp": Chirp()
        }
        
    def get_parameters(self) -> dict:
        return {
            "mass": self.config.mass,
            "length": self.config.length,
            "inertia": self.config.mass * self.config.length**2 / 3,
            "vin": self.config.vin,
            "offset": self.config.offset
        }

    def get_safety_limits(self) -> dict:
        return {
            "position_min": -np.deg2rad(96.5),
            "position_max": np.deg2rad(96.5)
        }
    
    def validate_trajectory(self, trajectory, dt=0.01) -> bool:
        """Validates if a trajectory stays within safety limits
        Args:
            trajectory: PendulumTrajectory instance
            dt: Time step for numerical derivatives
        Returns:
            bool: True if trajectory is safe, False otherwise
        Raises:
            ValueError: With detailed information about limit violations
        """
        limits = self.get_safety_limits()
        t = 0.0
        
        while t <= trajectory.duration:
            pos, _ = trajectory(t)
            
            # Check position limits
            if not (limits["position_min"] <= pos <= limits["position_max"]):
                raise ValueError(
                    f"Position limit exceeded at t={t:.2f}s: "
                    f"{np.rad2deg(pos):.1f}° (limits: "
                    f"{np.rad2deg(limits['position_min']):.1f}° to "
                    f"{np.rad2deg(limits['position_max']):.1f}°)"
                )
                
            t += dt
            
        return True


    async def read_state(self) -> Dict:
        """Read current state of the system
        Returns state in radians/rad per sec for positions/velocities"""
        # Create a fresh KOS connection for this read
        response = await self.kos.actuator.get_actuators_state([self.config.actuator_id])
        state = response.states[0]
        
        return {
            "position": np.deg2rad(float(state.position)),
            "speed": np.deg2rad(float(state.velocity)),
            "torque": float(state.torque),
            "input_volts": float(state.voltage),
            "temp": float(state.temperature),
            "current": float(state.current),
            "load": 0
        }
        
        return converted
    
    async def command_state(self, state: Dict):
        """Command system state
        Expects position in radians"""
        await self.kos.actuator.command_actuators([{
            'actuator_id': self.config.actuator_id,
            'position': np.rad2deg(state['position'])  # Convert to degrees for KOS
        }])

    def run_experiment(self, trajectory_name: str) -> Dict:
        """Run experiment with named trajectory"""
        return asyncio.run(self._run_experiment(trajectory_name))
    
    async def _run_experiment(self, trajectory_name: str) -> Dict:
        """Run experiment with named trajectory
        All internal calculations and storage use radians"""
        if trajectory_name not in self.trajectories:
            raise ValueError(f"Unknown trajectory: {trajectory_name}. " +
                           f"Available trajectories: {list(self.trajectories.keys())}")
        
        trajectory = self.trajectories[trajectory_name]

        if not self.validate_trajectory(trajectory):
            raise ValueError("Trajectory is not safe. Please adjust the trajectory.")
      
        await asyncio.sleep(1)
        current_state = await self.read_state()
        current_position = current_state['position']
        await self.command_state({"position": current_position})
        await asyncio.sleep(1)

        current_position = current_state['position']
        start_position = 0.0 +self.config.offset
        print(np.rad2deg(current_position))

        # Configure actuator
        await self.kos.actuator.configure_actuator(
            actuator_id=self.config.actuator_id,
            kp=self.config.kp,
            kd=0.0,
            ki=0.0,
            max_torque=self.config.max_torque,
            acceleration=0.0,
            torque_enabled=True
        )
        await asyncio.sleep(0.1)
        current_torque_state = True
        print("Torque enabled:", current_torque_state)

        # Move to starting position smoothly
        #start_position = trajectory(0)[0] + np.deg2rad(self.config.offset)  # Get initial position from trajectory

        current_state = await self.read_state()
        current_position = current_state['position']
        start_position = 0.0 +self.config.offset
        print(np.rad2deg(current_position))
        
        # Create keyframes for smooth motion to start position (2 second move)
        move_duration = 3.0
        keyframes = [
            [0.0, current_position, 0.0],  # [time, position, velocity]
            [move_duration, start_position, 0.0]
        ]

        print(f"Moving from {np.rad2deg(current_position)} to start position {np.rad2deg(start_position):.1f} degrees...")
        start_time = asyncio.get_running_loop().time()
        dt = 1.0 / self.config.sample_rate
        
        while asyncio.get_running_loop().time() - start_time < move_duration:
            t = asyncio.get_running_loop().time() - start_time
            position = PendulumTrajectory.cubic_interpolate(None, keyframes, t)
            await self.command_state({"position": position})
            await asyncio.sleep(dt)

        # Run experiment and collect data
        data = {

            # System parameters
            "mass": self.config.mass,
            "length": self.config.length,
            "vin": self.config.vin,
            "offset": self.config.offset,
            # Controller parameters
            "kp": self.config.kp,
            "kd": 0.0,
            "ki": 0.0,
            "max_torque": self.config.max_torque,
            # Hardware config
            "motor": self.config.motor,
            "actuator_id": self.config.actuator_id,
            "ip": self.config.ip,
            "sample_rate": self.config.sample_rate,
            # Experiment info
            "trajectory": trajectory_name,

            "entries": []
        }

        
        start_time = asyncio.get_running_loop().time()
        next_sample_time = start_time
        current_torque_state = True  # Track current torque state

       

        print(f"Running experiment for {trajectory.duration} seconds")
        while asyncio.get_running_loop().time() - start_time < trajectory.duration:
            t = asyncio.get_running_loop().time() - start_time
            goal_position, torque_enable = trajectory(t)
            
            # Hack for Lift and Drop and (bug in KOS)
            if trajectory_name == "lift_and_drop" and not torque_enable:
                await self.kos.actuator.configure_actuator(
                    actuator_id=self.config.actuator_id,
                    torque_enabled=False
                )
            elif torque_enable != current_torque_state:
                await self.kos.actuator.configure_actuator(
                    actuator_id=self.config.actuator_id,
                    torque_enabled=torque_enable
                )
                print("Torque enabled:", torque_enable)
                await asyncio.sleep(0.1)
                current_torque_state = torque_enable
                
            await self.command_state({"position": goal_position + self.config.offset})
            entry = await self.read_state()
            entry.update({
                "timestamp": t,
                "goal_position": goal_position,
                "torque_enable": torque_enable,
            })
            data["entries"].append(entry)
            
            # Timing compensation
            next_sample_time += dt
            sleep_time = next_sample_time - asyncio.get_running_loop().time()
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
            else:
                if -sleep_time*1000.0 > 2.0:
                    print(f"Warning: Falling behind schedule by {-sleep_time*1000:.1f}ms")
            

        # Filter out Position and Velocity spikes
        if len(data["entries"]) > 0:
            # Extract time series data
            times = np.array([entry["timestamp"] for entry in data["entries"]])
            positions = np.array([entry["position"] for entry in data["entries"]])
            velocities = np.array([entry["speed"] for entry in data["entries"]])
            
            # Convert to degrees for filtering
            positions_deg = np.rad2deg(positions)
            velocities_deg = np.rad2deg(velocities)
            
            # Filter spikes
            filtered_pos_deg, filtered_vel_deg = detect_and_filter_spikes(
                positions_deg, 
                velocities_deg, 
                times
            )
            
            # Convert back to radians and update data
            for i, entry in enumerate(data["entries"]):
                entry["position"] = np.deg2rad(filtered_pos_deg[i])
                entry["speed"] = np.deg2rad(filtered_vel_deg[i])

        # Analyze data quality
        data_metrics = metrics.analyze_sysid_data(data)
        print("\n=== System ID Data Analysis ===")
        print(f"Trajectory: {data_metrics['trajectory_type']}")
        print(f"\nSampling Statistics:")
        print(f"Total Samples: {data_metrics['total_samples']}")
        print(f"Duration: {data_metrics['duration']:.2f}s")
        print(f"Sample Rate: {data_metrics['actual_rate']:.1f}Hz (target: {data_metrics['target_rate']}Hz)")
        print(f"dt mean: {data_metrics['dt_mean']*1000:.2f}ms, std: {data_metrics['dt_std']*1000:.2f}ms")
        print(f"dt range: [{data_metrics['dt_min']*1000:.2f}, {data_metrics['dt_max']*1000:.2f}]ms")
        print(f"Missing samples: {data_metrics['missing_samples']}")

        plots_dir = Path("./plots")
        plots_dir.mkdir(exist_ok=True, parents=True)
        plotter = PendulumPlot(data)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        plotter.create_plots(save_dir="./plots", timestamp=timestamp)

        return data