# ktune/core/tune.py
import asyncio
import math
import time
import json
import os
import numpy as np
from datetime import datetime
import matplotlib.pyplot as plt
import logging
from dataclasses import dataclass
from typing import Dict, List, Optional
from pykos import KOS
from ktune.core.utils.datalog import DataLog
from ktune.core.utils.plots import Plot
from ktune.core.utils import metrics
# Configure logging
logging.getLogger('matplotlib').setLevel(logging.WARNING)
os.environ["PYTHONWARNINGS"] = "ignore"
logging.getLogger().setLevel(logging.ERROR)

@dataclass
class TuneConfig:
    """Configuration for tuning tests"""
    # Connection settings
    name: str = "NoName"
    mode: str = "compare"  # Add mode parameter with default
    sim_ip: str = "127.0.0.1"
    real_ip: str = "192.168.42.1"
    actuator_id: int = 11
    start_pos: float = 0.0
    
    # Actuator gains
    kp: float = 20.0
    kd: float = 5.0
    ki: float = 0.0
    
    # Actuator config
    acceleration: float = 0.0
    max_torque: float = 100.0
    torque_off: bool = False

    # Simulation gains
    sim_kp: float = 20.0
    sim_kd: float = 5.0
    stream_delay: float = 0.0

    # Logging config
    no_log: bool = False
    log_duration_pad: float = 2.0
    sample_rate: float = 100.0

    # Servo control
    enable_servos: Optional[List[int]] = None
    disable_servos: Optional[List[int]] = None

    # Test parameters (will be set by specific test commands)
    test: Optional[str] = None
    # Sine parameters
    freq: Optional[float] = None
    amp: Optional[float] = None
    duration: Optional[float] = None
    # Step parameters
    step_size: Optional[float] = None
    step_hold_time: Optional[float] = None
    step_count: Optional[int] = None
    # Chirp parameters
    chirp_amp: Optional[float] = None
    chirp_init_freq: Optional[float] = None
    chirp_sweep_rate: Optional[float] = None
    chirp_duration: Optional[float] = None

class Tune:
    def __init__(self, config: Dict):
        tune_config = config.get('tune', {})
        self.config = TuneConfig(**tune_config)
        self.mode = self.config.mode
        
        # Initialize data storage based on mode
        self.sim_data = None
        self.real_data = None
        
        if self.mode in ['compare', 'sim']:
            self.sim_data = {
                "time": [], "position": [], "velocity": [],
                "cmd_time": [], "cmd_pos": [], "cmd_vel": []
            }
        if self.mode in ['compare', 'real']:
            self.real_data = {
                "time": [], "position": [], "velocity": [],
                "cmd_time": [], "cmd_pos": [], "cmd_vel": []
            }


    async def setup_connections(self):
        """Initialize and test connections based on mode"""
        if self.mode in ['compare', 'sim']:
            print("Testing KOS-SIM connection...")
            # For sim mode, use sim_ip as the connection address
            sim_ip = self.config.sim_ip
            self.sim_kos = KOS(sim_ip)
            sim_start = time.time()
            for _ in range(100):
                await self.sim_kos.actuator.get_actuators_state([self.config.actuator_id])
            sim_rate = 100 / (time.time() - sim_start)
            print(f"Max KOS-SIM sampling rate: {sim_rate:.1f} Hz")
                
        if self.mode in ['compare', 'real']:
            print("Testing KOS-REAL connection...")
            # For real mode, use real_ip as the connection address
            real_ip = self.config.real_ip
            self.real_kos = KOS(real_ip)
            real_start = time.time()
            for _ in range(100):
                await self.real_kos.actuator.get_actuators_state([self.config.actuator_id])
            real_rate = 100 / (time.time() - real_start)
            print(f"Max KOS-REAL sampling rate: {real_rate:.1f} Hz")

        print(f"Required sampling rate: {self.config.sample_rate} Hz")
        if ((self.mode in ['compare', 'sim'] and sim_rate < self.config.sample_rate) or
            (self.mode in ['compare', 'real'] and real_rate < self.config.sample_rate)):
            raise ValueError(
                f"Requested sampling rate ({self.config.sample_rate} Hz) exceeds "
                "maximum achievable rates. Try re-running with --no-render or "
                "reduce the sampling rate"
            )
        
    def _print_test_config(self):
        """Print test configuration and motor settings."""
        print("\nTest Configuration:")
        print(f"Test Type: {self.config.test}")
        print(f"Actuator ID: {self.config.actuator_id}")
        print(f"Start Position: {self.config.start_pos}°")
        print(f"Sample Rate: {self.config.sample_rate} Hz")
        
        print("\nMotor Settings:")
        print("Real System:")
        print(f"  Kp: {self.config.kp}")
        print(f"  Kd: {self.config.kd}")
        print(f"  Ki: {self.config.ki}")
        print(f"  Max Torque: {self.config.max_torque}")
        print(f"  Acceleration: {self.config.acceleration}")
        print("Simulation:")
        print(f"  Kp: {self.config.sim_kp}")
        print(f"  Kd: {self.config.sim_kd}")
        
        # Print test-specific parameters
        if self.config.test == "step":
            print("\nStep Test Parameters:")
            print(f"Step Size: {self.config.step_size}°")
            print(f"Hold Time: {self.config.step_hold_time}s")
            print(f"Step Count: {self.config.step_count}")
            total_duration = (self.config.step_hold_time * (2 * self.config.step_count + 1) + 
                            self.config.log_duration_pad)
            print(f"Total Duration: {total_duration}s")
        
        elif self.config.test == "sine":
            print("\nSine Test Parameters:")
            print(f"Frequency: {self.config.freq} Hz")
            print(f"Amplitude: {self.config.amp}°")
            print(f"Duration: {self.config.duration}s")
            print(f"Total Duration: {self.config.duration + self.config.log_duration_pad}s")
        
        elif self.config.test == "chirp":
            print("\nChirp Test Parameters:")
            print(f"Initial Frequency: {self.config.chirp_init_freq} Hz")
            print(f"Sweep Rate: {self.config.chirp_sweep_rate} Hz/s")
            print(f"Amplitude: {self.config.chirp_amp}°")
            print(f"Duration: {self.config.chirp_duration}s")
            print(f"Total Duration: {self.config.chirp_duration + self.config.log_duration_pad}s")

    def run_test(self, test_type: Optional[str] = None):
        """Main entry point for running tests"""
        if test_type is None and not (self.config.enable_servos or self.config.disable_servos):
            raise ValueError("No test type specified and no servo operations requested")
        
        asyncio.run(self._run_test(test_type))
        
        # Only save and plot if we ran a test
        if test_type is not None:
            self.save_and_plot_results()

    async def _run_test(self, test_type: Optional[str] = None):
        """Async implementation of test execution"""
        if test_type is not None:
            await self.setup_connections()
        else:
            # Simple connection without sampling rate test
            if self.mode in ['compare', 'sim']:
                self.sim_kos = KOS(self.config.sim_ip)
            if self.mode in ['compare', 'real']:
                self.real_kos = KOS(self.config.real_ip)
        
        # Configure servos if needed
        if self.config.enable_servos:
            await self._enable_servos(self.config.enable_servos)
        if self.config.disable_servos:
            await self._disable_servos(self.config.disable_servos)

        # Run the specified test
        if test_type is None:
            print("No test specified, exiting after servo configuration.")
            return
            
        if test_type == "sine":
            await self._run_sine_test()
        elif test_type == "step":
            await self._run_step_test()
        elif test_type == "chirp":
            await self._run_chirp_test()
        else:
            print(f"Unknown test type '{test_type}', exiting.")
            return

        # Clean up connections
        if hasattr(self, 'sim_kos'):
            await self.sim_kos.close()
        if hasattr(self, 'real_kos'):
            await self.real_kos.close()

    async def _disable_servos(self, servo_ids=None):
        """Disable servos based on mode"""
        ids_to_disable = servo_ids if servo_ids else self.config.servo_ids

        if self.mode in ['compare', 'sim']:
            print("Disabling simulation servos...")
            for servo_id in ids_to_disable:
                await self.sim_kos.actuator.configure_actuator(
                    actuator_id=servo_id,
                    torque_enabled=False
                )
        
        if self.mode in ['compare', 'real']:
            print("Disabling real servos...")
            for servo_id in ids_to_disable:
                await self.real_kos.actuator.configure_actuator(
                    actuator_id=servo_id,
                    torque_enabled=False
                )

    async def _enable_servos(self, servo_ids=None):
        """Enable servos based on mode"""
        ids_to_enable = servo_ids if servo_ids else self.config.servo_ids

        if self.mode in ['compare', 'sim']:
            print("Enabling simulation servos...")
            for servo_id in ids_to_enable:
                await self.sim_kos.actuator.configure_actuator(
                    actuator_id=servo_id,
                    torque_enabled=True
                )
        
        if self.mode in ['compare', 'real']:
            print("Enabling real servos...")
            for servo_id in ids_to_enable:
                await self.real_kos.actuator.configure_actuator(
                    actuator_id=servo_id,
                    torque_enabled=True
                )

    async def _move_to_start_position(self, kos_configs):
        """Move to start position and wait until position is reached.
        
        Args:
            kos_configs: List of (KOS, is_real) tuples for active systems
        """
        print(f"\nMoving to start position: {self.config.start_pos}°")
        
        # Command move to start position
        for kos, _ in kos_configs:
            print(f"Moving to start position: {self.config.start_pos}°")
            await kos.actuator.command_actuators([{
                'actuator_id': self.config.actuator_id,
                'position': self.config.start_pos,
            }])
            await asyncio.sleep(1.0)

        # Wait for position to be reached
        settling_time = 1
        max_settling_time = 10.0  # Maximum time to wait for settling
        position_threshold = 0.2  # degrees
        
        while settling_time < max_settling_time:
            all_settled = True
            
            for kos, is_real in kos_configs:
                response = await kos.actuator.get_actuators_state([self.config.actuator_id])
                if response.states:
                    current_pos = response.states[0].position
                    error = abs(current_pos - self.config.start_pos)
                    system_type = "Real" if is_real else "Sim"
                    if error > position_threshold:
                        all_settled = False
                        print(f"{system_type} Position Error: {error:.3f}°", end='\r')
            
            if all_settled:
                print("\nStart position reached!")
                break
                
            settling_time += 0.1
            await asyncio.sleep(0.1)
        
        if settling_time >= max_settling_time:
            print("\nWarning: Start position not reached within timeout")

        await asyncio.sleep(1.0)
                
    def _log_actuator_state(self, response, data_dict, current_time):
        """Log actuator state data with normalized time.
        
        Args:
            response: Actuator state response
            data_dict: Dictionary to store data
            current_time: Current normalized time (seconds from start)
        """
        if response.states:
            state = response.states[0]
            log_time = current_time
            if data_dict is self.sim_data:
                log_time = current_time + self.config.stream_delay
            if state.position is not None:
                data_dict["position"].append(state.position)
            if state.velocity is not None:
                data_dict["velocity"].append(state.velocity)
            data_dict["time"].append(log_time)

    

    async def _run_step_test(self):
        """Run step response test on both sim and real systems"""
        self._print_test_config()
        # Construct step sequence
        vel = 0.0  # Default velocity limit
        steps = [(0.0, vel, self.config.step_hold_time)]
        for _ in range(self.config.step_count):
            steps.append((self.config.step_size, vel, self.config.step_hold_time))
            steps.append((0.0, vel, self.config.step_hold_time))

        # Calculate total duration including padding
        total_duration = (sum(step[2] for step in steps) + 
                        self.config.log_duration_pad)

        # Configure actuators based on mode
        kos_configs = []
        if self.mode in ['compare', 'sim']:
            kos_configs.append((self.sim_kos, False))
        if self.mode in ['compare', 'real']:
            kos_configs.append((self.real_kos, True))

        # Configure each active KOS instance
        for kos, is_real in kos_configs:
            # Select gains based on system type
            if is_real:
                kp, kd, ki = (self.config.kp, self.config.kd, self.config.ki)
            else:
                kp, kd, ki = (self.config.sim_kp, self.config.sim_kd, 0.0)
            print(f"kp: {kp}, kd: {kd}, ki: {ki} is_real: {is_real}")
            await kos.actuator.configure_actuator(
                actuator_id=self.config.actuator_id,
                kp=kp, kd=kd, ki=ki,
                acceleration=self.config.acceleration,
                max_torque=self.config.max_torque,
                torque_enabled=not self.config.torque_off
            )

        # Move to start position and wait for settling
        await self._move_to_start_position(kos_configs)

        # Start test
        start_time = time.time()
        current_time = 0.0
        step_idx = 0

        while current_time < total_duration:
            current_time = time.time() - start_time

            # Determine current step target
            while (step_idx < len(steps) and 
                current_time > sum(step[2] for step in steps[:step_idx + 1])):
                step_idx += 1

            if step_idx < len(steps):
                target_pos = steps[step_idx][0] + self.config.start_pos

                # Command active systems
                for kos, is_real in kos_configs:
                    data_dict = self.real_data if is_real else self.sim_data
                    # Send command
                    await kos.actuator.command_actuators([{
                        'actuator_id': self.config.actuator_id,
                        'position': target_pos,
                    }])
                    
                    # Log command
                    data_dict["cmd_time"].append(current_time)
                    data_dict["cmd_pos"].append(target_pos)
                    data_dict["cmd_vel"].append(0.0)  # No velocity command for steps

                    # Get and log state
                    response = await kos.actuator.get_actuators_state(
                        [self.config.actuator_id]
                    )
                    self._log_actuator_state(response, data_dict, current_time)

            await asyncio.sleep(1.0 / self.config.sample_rate)


    async def _run_sine_test(self):
        """Run sine wave test on both sim and real systems"""
        self._print_test_config()

        # Calculate total duration including padding
        total_duration = self.config.duration + self.config.log_duration_pad

        # Configure actuators based on mode
        kos_configs = []
        if self.mode in ['compare', 'sim']:
            kos_configs.append((self.sim_kos, False))
        if self.mode in ['compare', 'real']:
            kos_configs.append((self.real_kos, True))

        # Configure each active KOS instance
        for kos, is_real in kos_configs:
            # Select gains based on system type
            if is_real:
                kp, kd, ki = (self.config.kp, self.config.kd, self.config.ki)
            else:
                kp, kd, ki = (self.config.sim_kp, self.config.sim_kd, 0.0)

            await kos.actuator.configure_actuator(
                actuator_id=self.config.actuator_id,
                kp=kp, kd=kd, ki=ki,
                acceleration=self.config.acceleration,
                max_torque=self.config.max_torque,
                torque_enabled=not self.config.torque_off
            )

        # Move to start position and wait for settling
        await self._move_to_start_position(kos_configs)

        # Start test
        start_time = time.time()
        current_time = 0.0

        while current_time < total_duration:
            current_time = time.time() - start_time

            if current_time <= self.config.duration:
                # Calculate sine wave position and velocity
                omega = 2.0 * math.pi * self.config.freq
                phase = omega * current_time
                
                target_pos = (self.config.amp * math.sin(phase) + 
                            self.config.start_pos)
                target_vel = self.config.amp * omega * math.cos(phase)

                # Command active systems
                for kos, is_real in kos_configs:
                    data_dict = self.real_data if is_real else self.sim_data
                    # Send command with both position and velocity
                    await kos.actuator.command_actuators([{
                        'actuator_id': self.config.actuator_id,
                        'position': target_pos,
                    }])
                    
                    # Log command
                    data_dict["cmd_time"].append(current_time)
                    data_dict["cmd_pos"].append(target_pos)
                    data_dict["cmd_vel"].append(target_vel)

                    # Get and log state
                    response = await kos.actuator.get_actuators_state(
                        [self.config.actuator_id]
                    )
                    self._log_actuator_state(response, data_dict, current_time)

            await asyncio.sleep(1.0 / self.config.sample_rate)

        # Calculate tracking metrics only for active systems
        if self.mode in ['compare', 'sim']:
            sim_error = metrics.compute_tracking_error(
                self.sim_data["cmd_time"],
                self.sim_data["cmd_pos"],
                self.sim_data["time"],
                self.sim_data["position"]
            )
            print(f"Sim RMS Error: {sim_error:.3f}°")

        if self.mode in ['compare', 'real']:
            real_error = metrics.compute_tracking_error(
                self.real_data["cmd_time"],
                self.real_data["cmd_pos"],
                self.real_data["time"],
                self.real_data["position"]
            )
            print(f"Real RMS Error: {real_error:.3f}°")

    async def _run_chirp_test(self):
        """Run chirp test on both sim and real systems"""
        self._print_test_config()
        # Calculate total duration including padding
        total_duration = self.config.chirp_duration + self.config.log_duration_pad

        # Configure actuators based on mode
        kos_configs = []
        if self.mode in ['compare', 'sim']:
            kos_configs.append((self.sim_kos, False))
        if self.mode in ['compare', 'real']:
            kos_configs.append((self.real_kos, True))

        # Configure each active KOS instance
        for kos, is_real in kos_configs:
            # Select gains based on system type
            if is_real:
                kp, kd, ki = (self.config.kp, self.config.kd, self.config.ki)
            else:
                kp, kd, ki = (self.config.sim_kp, self.config.sim_kd, 0.0)
            print(f"acceleration: {self.config}******************")
            config = {
                'torque_enabled': not self.config.torque_off,
                'kp': kp,
                'kd': kd,
                'ki': ki,
                'max_torque': self.config.max_torque,
                'acceleration': self.config.acceleration  # Make sure this is included
            }
            await kos.actuator.configure_actuator(
                actuator_id=self.config.actuator_id,
                **config
            )

        # Move to start position and wait for settling
        await self._move_to_start_position(kos_configs)

        # Start test
        start_time = time.time()
        current_time = 0.0

        while current_time < total_duration:
            current_time = time.time() - start_time

            if current_time <= self.config.chirp_duration:
                # Calculate chirp signal
                f0 = self.config.chirp_init_freq
                k = self.config.chirp_sweep_rate
                phase = 2.0 * math.pi * (f0 * current_time + 
                                    0.5 * k * current_time * current_time)
                
                # Instantaneous frequency and angular velocity
                freq = f0 + k * current_time
                omega = 2.0 * math.pi * freq
                
                # Calculate position and velocity
                target_pos = (self.config.chirp_amp * math.sin(phase) + 
                            self.config.start_pos)
                target_vel = self.config.chirp_amp * omega * math.cos(phase)

                # Command active systems
                for kos, is_real in kos_configs:
                    data_dict = self.real_data if is_real else self.sim_data
                    # Send command with both position and velocity
                    await kos.actuator.command_actuators([{
                        'actuator_id': self.config.actuator_id,
                        'position': target_pos,
                    }])
                    
                    # Log command
                    data_dict["cmd_time"].append(current_time)
                    data_dict["cmd_pos"].append(target_pos)
                    data_dict["cmd_vel"].append(target_vel)

                    # Get and log state
                    response = await kos.actuator.get_actuators_state(
                        [self.config.actuator_id]
                    )
                    self._log_actuator_state(response, data_dict, current_time)

            await asyncio.sleep(1.0 / self.config.sample_rate)
        
        # Compute frequency response only for active systems
        if self.mode in ['compare', 'sim']:
            try:
                freq_response = metrics.analyze_frequency_response(self.sim_data)
                sim_freq_response = freq_response.get('sim', {})
                self.sim_data["freq_response"] = sim_freq_response
                print("\nSim Frequency Response Data:")
                if sim_freq_response:
                    print(f"Frequencies: {len(sim_freq_response.get('freq', []))}")
                    sim_mag = sim_freq_response.get('magnitude', [])
                    if sim_mag:
                        print(f"Magnitude range: {min(sim_mag)} to {max(sim_mag)}")
                        sim_bandwidth = metrics.compute_bandwidth(
                            sim_freq_response["freq"], 
                            sim_freq_response["magnitude"]
                        )
                        if sim_bandwidth:
                            print(f"Bandwidth (-3dB): {sim_bandwidth:.1f} Hz")
            except Exception as e:
                print(f"Warning: Could not compute simulation frequency response: {e}")

        if self.mode in ['compare', 'real']:
            try:
                freq_response = metrics.analyze_frequency_response(self.real_data)
                real_freq_response = freq_response.get('real', {})
                self.real_data["freq_response"] = real_freq_response
                print("\nReal Frequency Response Data:")
                if real_freq_response:
                    print(f"Frequencies: {len(real_freq_response.get('freq', []))}")
                    real_mag = real_freq_response.get('magnitude', [])
                    if real_mag:
                        print(f"Magnitude range: {min(real_mag)} to {max(real_mag)}")
                        real_bandwidth = metrics.compute_bandwidth(
                            real_freq_response["freq"], 
                            real_freq_response["magnitude"]
                        )
                        if real_bandwidth:
                            print(f"Bandwidth (-3dB): {real_bandwidth:.1f} Hz")
            except Exception as e:
                print(f"Warning: Could not compute real system frequency response: {e}")

    def save_and_plot_results(self):
        """Save data to files and generate plots"""
        if self.config.no_log:
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        data_dir = os.path.join(os.getcwd(), "data")
        plot_dir = os.path.join(os.getcwd(), "plots")
        os.makedirs(data_dir, exist_ok=True)
        os.makedirs(plot_dir, exist_ok=True)

        # Only pass data that exists based on mode
        sim_data = self.sim_data if self.mode in ['compare', 'sim'] else {}  # Empty dict instead of None
        real_data = self.real_data if self.mode in ['compare', 'real'] else {}  # Empty dict instead of None

        # Save data
        logger = DataLog(self.config, sim_data, real_data)
        logger.save_data(timestamp, data_dir)

        # Create plots
        plotter = Plot(self.config, sim_data, real_data)
        plotter.create_plots(timestamp, plot_dir)

