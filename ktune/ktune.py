#!/usr/bin/env python3
"""
ktune - A CLI tool for tuning Sim2Real
    # See --help for all options.
"""

import argparse
import asyncio
from multiprocessing import Process, Queue
import math
import time
import json
import os
import numpy as np
from datetime import datetime
import matplotlib.pyplot as plt
import logging
from pykos import KOS
from . import __version__

os.environ["PYTHON_IMK_OVERRIDE"] = "1"
# Suppress gRPC fork warnings
os.environ["GRPC_ENABLE_FORK_SUPPORT"] = "false"
# Suppress matplotlib/IMK logging
logging.getLogger('matplotlib').setLevel(logging.WARNING)
os.environ["PYTHONWARNINGS"] = "ignore"

logging.getLogger().setLevel(logging.ERROR)

############################
# ACTUATOR STATE LOGGING   #
############################
def log_actuator_state(response,data_dict):
    if response.states:
        state = response.states[0]
        if state.position is not None:
            data_dict["position"].append(state.position)
        if state.velocity is not None:
            data_dict["velocity"].append(state.velocity)
        if state.torque is not None:
            data_dict["torque"].append(state.torque)
        if state.voltage is not None:
            data_dict["voltage"].append(state.voltage)
        if state.current is not None:
            data_dict["current"].append(state.current)


#############################
# TUNING METRICS  #
#############################
def compute_step_overshoots(time_array, pos_array, steps, window_duration=1.0):
    """
    For each step transition, compute the overshoot percentage based on the maximum (or minimum)
    position reached within a fixed window (default: 1 second) after the new target is commanded.

    For an upward step (new_target > old_target):
      overshoot (%) = (peak - new_target) / (new_target - old_target) * 100
    For a downward step (new_target < old_target):
      overshoot (%) = (new_target - trough) / (old_target - new_target) * 100

    :param time_array: Array of time stamps (seconds)
    :param pos_array:  Array of measured positions (degrees)
    :param steps:      List of tuples (target, duration) that define the step sequence.
                       The first element defines the starting position.
    :param window_duration: Duration (in seconds) after the new command to look for the peak.
    :return: List of overshoot percentages (one per transition).
    """
    import numpy as np

    time_array = np.array(time_array)
    pos_array  = np.array(pos_array)

    # Build an array of step command times (cumulative durations)
    step_times = [0.0]
    for (target, velocity, duration) in steps:
        step_times.append(step_times[-1] + duration)

    overshoots = []
    # For each step transition (i.e. from step i-1 to i)
    for i in range(1, len(steps)):
        old_target = steps[i-1][0]
        new_target = steps[i][0]
        command_time = step_times[i]
        window_end = command_time + window_duration

        # Find indices of data within [command_time, command_time + window_duration]
        idx = np.where((time_array >= command_time) & (time_array <= window_end))[0]
        if len(idx) == 0:
            continue
        p_window = pos_array[idx]

        if new_target > old_target:
            # For an upward step, overshoot is defined by the maximum value in the window.
            peak = np.nanmax(p_window)
            overshoot = (peak - new_target) / (new_target - old_target) * 100.0
        else:
            # For a downward step, overshoot is defined by the minimum value in the window.
            trough = np.nanmin(p_window)
            overshoot = (new_target - trough) / (old_target - new_target) * 100.0

        # Clamp negative overshoot to zero.
        overshoots.append(max(0.0, overshoot))

    return overshoots


#############################
# ACTUATOR TEST FUNCTIONS   $
# CHIRP                     #
#############################
async def run_chirp_test(kos: KOS,
                         actuator_id: int,
                         amplitude: float,
                         init_freq: float,
                         sweep_rate: float,
                         duration: float,
                         kp: float,
                         kd: float,
                         ki: float,
                         sim_kp: float,
                         sim_kv: float,
                         acceleration: float,
                         max_torque: float,
                         torque_enabled: bool,
                         update_rate: float,
                         data_dict: dict,
                         start_time: float,
                         is_real: bool,
                         start_pos: float = 0.0):
    """
    Command a chirp waveform and log timestamps, commanded, and measured values.
    The chirp is defined as:
       angle = amplitude * sin(2*pi*(init_freq*t + 0.5*sweep_rate*t^2))
       velocity = amplitude * cos(2*pi*(init_freq*t + 0.5*sweep_rate*t^2)) * 2*pi*(init_freq + sweep_rate*t)
    """
    # Choose gains based on whether we're on a real system or simulation.
    if is_real:
        used_kp, used_kd, used_ki = kp, kd, ki
    else:
        used_kp, used_kd, used_ki = sim_kp, sim_kv, 0

    # Configure the actuator.
    await kos.actuator.configure_actuator(
        actuator_id=actuator_id,
        kp=used_kp, kd=used_kd, ki=used_ki,
        acceleration=acceleration,
        max_torque=max_torque,
        torque_enabled=torque_enabled
    )

    print(f"Chirp Test | Real: {is_real}, Init Freq: {init_freq} Hz, Sweep Rate: {sweep_rate} Hz/s, Amp: {amplitude}°, Duration: {duration}s")

    await kos.actuator.command_actuators([
        {'actuator_id': actuator_id, 'position': start_pos} #, 'velocity': 5}
    ])
    await asyncio.sleep(2)
    start_time = time.time()

    dt = 1.0 / update_rate
    steps = int(duration / dt)
    next_tick = time.time()
    for i in range(steps):
        t = i * dt
        # Compute phase, angle and velocity.
        phase = 2 * math.pi * (init_freq * t + 0.5 * sweep_rate * t * t)
        angle = start_pos + amplitude * math.sin(phase)
        vel = amplitude * math.cos(phase) * 2 * math.pi * (init_freq + sweep_rate * t)

        t_send = time.time() - start_time
        data_dict["cmd_time"].append(t_send)
        data_dict["cmd_pos"].append(angle)
        data_dict["cmd_vel"].append(vel)

        await kos.actuator.command_actuators([
            {'actuator_id': actuator_id, 'position': angle} #, 'velocity': vel}
        ])

        response = await kos.actuator.get_actuators_state([actuator_id])
        t_resp = time.time() - start_time
        log_actuator_state(response=response,data_dict=data_dict)
        data_dict["time"].append(t_resp)

        next_tick += dt
        sleep_time = next_tick - time.time()
        if sleep_time > 0:
            await asyncio.sleep(sleep_time)


#############################
# ACTUATOR TEST FUNCTIONS   $
# SINUSOID                  #
#############################
async def run_sine_test(kos: KOS,
                        actuator_id: int,
                        amplitude: float,
                        freq: float,
                        duration: float,
                        kp: float,
                        kd: float,
                        ki: float,
                        sim_kp: float,
                        sim_kv: float,
                        acceleration: float,
                        max_torque: float,
                        torque_enabled: bool,
                        update_rate: float,
                        data_dict: dict,
                        start_time: float,
                        is_real: bool,
                        start_pos: float = 0.0):
    """
    Command a sine wave and log timestamps, commanded, and measured values.
    Uses simulation gains (sim_kp, sim_kv) if is_real is False.
    When request_state is False, measured values are set equal to commanded values.
    """
    # Choose which gains to use
    if is_real:
        used_kp = kp
        used_kd = kd
        used_ki = ki
    else:
        used_kp = sim_kp
        used_kd = sim_kv
        used_ki = 0

    # Configure actuator
    if is_real:
        await kos.actuator.configure_actuator(
            actuator_id=actuator_id,
            kp=used_kp, kd=used_kd, ki=used_ki,
            acceleration=acceleration,
            max_torque=max_torque,
            torque_enabled=torque_enabled
        )
    else:
        await kos.actuator.configure_actuator(
            actuator_id=actuator_id,
            kp=used_kp, kd=used_kd,
            max_torque=max_torque,
            torque_enabled=torque_enabled
        )

    print(f"Real: {is_real}, Frequency: {freq}, Amplitude: {amplitude}, Duration: {duration}")

    await kos.actuator.command_actuators([
        {'actuator_id': actuator_id, 'position': start_pos} #, 'velocity': 2}
    ])
    await asyncio.sleep(2)
    start_time = time.time()

    dt = 1.0 / update_rate
    steps = int(duration / dt)
    next_tick = time.time()
    for i in range(steps):
        t = i * dt
        angle = start_pos + amplitude * math.sin(2.0 * math.pi * freq * t)
        vel = amplitude * (2.0 * math.pi * freq) * math.cos(2.0 * math.pi * freq * t)

        # Log command time
        t_send = time.time() - start_time
        data_dict["cmd_time"].append(t_send)
        data_dict["cmd_pos"].append(angle)
        data_dict["cmd_vel"].append(vel)
        
        # Send the command
        await kos.actuator.command_actuators([
            {'actuator_id': actuator_id, 'position': angle} #, 'velocity': vel}
        ])

        response = await kos.actuator.get_actuators_state([actuator_id])
        t_resp = time.time() - start_time
        log_actuator_state(response=response,data_dict=data_dict)
        data_dict["time"].append(t_resp)
                
        next_tick += dt
        sleep_time = next_tick - time.time()
        if sleep_time > 0:
            await asyncio.sleep(sleep_time)



#############################
# ACTUATOR TEST FUNCTIONS   #
# STEP                      #
#############################
# Updated run_step_test signature and configuration:
async def run_step_test(
    kos: KOS,
    actuator_id: int,
    step_size: float,
    step_hold_time: float,
    step_count: int,
    kp: float,
    kd: float,
    ki: float,
    sim_kp: float,
    sim_kv: float,
    acceleration: float,
    max_torque: float,
    torque_enabled: bool = True,
    vel_limit: float = 200.0,
    data_dict: dict = None,
    start_time: float = None,
    sample_rate: float = 50.0,
    is_real: bool = True,
    start_pos: float = 0.0):

    """
    Perform a step test with continuous sampling during hold periods.
    Uses simulation gains if is_real is False.
    """
    if is_real:
        used_kp = kp
        used_kd = kd
        used_ki = ki
    else:
        used_kp = sim_kp
        used_kd = sim_kv
        used_ki = ki
        
    # Configure the actuator
    await kos.actuator.configure_actuator(
        actuator_id=actuator_id,
        kp=used_kp, kd=used_kd, ki=used_ki,
        acceleration=acceleration,
        max_torque=max_torque,
        torque_enabled=torque_enabled
    )

    await kos.actuator.command_actuators([{'actuator_id': actuator_id, 'position': start_pos}])

    await asyncio.sleep(2)
    sample_period = 1.0 / sample_rate
    next_sample_time = time.time()

    async def sample_state(current_cmd_pos, current_cmd_vel):
        nonlocal next_sample_time
        try:
            response = await kos.actuator.get_actuators_state([actuator_id])
            t_resp = time.time() - start_time
            log_actuator_state(response=response, data_dict=data_dict)
            data_dict["time"].append(t_resp)
            data_dict["cmd_time"].append(t_resp)  # Record command at each sample
            data_dict["cmd_pos"].append(current_cmd_pos)
            data_dict["cmd_vel"].append(current_cmd_vel)
            
            next_sample_time += sample_period
            sleep_time = next_sample_time - time.time()
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
        except Exception as e:
            print(f"Error sampling state: {e}")


    start_time = time.time()
    end_hold = time.time() + step_hold_time
    while time.time() < end_hold:
        await sample_state(current_cmd_pos=start_pos, current_cmd_vel=vel_limit)
    
    for _ in range(step_count):
        # STEP UP        
        target_pos = start_pos + step_size
        await kos.actuator.command_actuators([{'actuator_id': actuator_id, 'position': target_pos}])
        end_hold = time.time() + step_hold_time
        while time.time() < end_hold:
            await sample_state(current_cmd_pos=target_pos, current_cmd_vel=vel_limit)
    
        # STEP DOWN
        target_pos = start_pos
        await kos.actuator.command_actuators([{'actuator_id': actuator_id, 'position': target_pos}])
        end_hold = time.time() + step_hold_time
        while time.time() < end_hold:
            await sample_state(current_cmd_pos=target_pos, current_cmd_vel=vel_limit)


#############################
# ENABLE/DISABLE SERVOS      #
#############################
async def configure_additional_servos(kos: KOS, args):
    # Enable servos from --enable-servos
    if args.enable_servos:
        enabled_ids = [int(x.strip()) for x in args.enable_servos.split(',') if x.strip()]
        for servo_id in enabled_ids:
            print(f"Enabling servo {servo_id}")
            await kos.actuator.configure_actuator(
                actuator_id=servo_id,
                torque_enabled=True
            )
    # Disable servos from --disable-servos
    if args.disable_servos:
        disabled_ids = [int(x.strip()) for x in args.disable_servos.split(',') if x.strip()]
        for servo_id in disabled_ids:
            print(f"Disabling servo {servo_id}")
            await kos.actuator.configure_actuator(
                actuator_id=servo_id,
                torque_enabled=False
            )



#############################
# SIMULATOR TEST #
#############################
def run_sim_test(args, global_start, out_queue):
    sim_data = {"time": [], "position": [], "velocity": [], "torque": [], "voltage": [], "current": [], "temperature": [], "cmd_time": [], "cmd_pos": [], "cmd_vel": []}
    if args.test == "sine":
        asyncio.run(run_sine_test(
            kos=KOS(args.sim_ip),
            actuator_id=args.actuator_id,
            amplitude=args.amp,
            freq=args.freq,
            duration=args.duration,
            kp=args.kp,
            kd=args.kd,
            ki=args.ki,
            sim_kp=args.sim_kp,
            sim_kv=args.sim_kv,
            acceleration=args.acceleration,
            max_torque=args.max_torque,
            torque_enabled=(not args.torque_off),
            update_rate=50.0,
            data_dict=sim_data,
            start_time=global_start,
            is_real=False,
            start_pos=args.start_pos,
        ))
    elif args.test == "step":
        asyncio.run(run_step_test(
            kos=KOS(args.sim_ip),
            actuator_id=args.actuator_id,
            step_size=args.step_size,
            step_hold_time=args.step_hold_time,
            step_count=args.step_count,
            kp=args.kp,
            kd=args.kd,
            ki=args.ki,
            sim_kp=args.sim_kp,
            sim_kv=args.sim_kv,
            acceleration=args.acceleration,
            max_torque=args.max_torque,
            torque_enabled=(not args.torque_off),
            vel_limit=400.0,
            data_dict=sim_data,
            start_time=global_start,
            sample_rate=args.sample_rate,
            is_real=False,
            start_pos=args.start_pos,
        ))
    elif args.test == "chirp":
        asyncio.run(run_chirp_test(
            kos=KOS(args.sim_ip),
            actuator_id=args.actuator_id,
            amplitude=args.chirp_amp,
            init_freq=args.chirp_init_freq,
            sweep_rate=args.chirp_sweep_rate,
            duration=args.chirp_duration,
            kp=args.kp,
            kd=args.kd,
            ki=args.ki,
            sim_kp=args.sim_kp,
            sim_kv=args.sim_kv,
            acceleration=args.acceleration,
            max_torque=args.max_torque,
            torque_enabled=(not args.torque_off),
            update_rate=50.0,
            data_dict=sim_data,
            start_time=global_start,
            is_real=False,
            start_pos=args.start_pos,
        ))
    out_queue.put(sim_data)


#############################
# REAL ROBOT TEST #
#############################
def run_real_test(args, global_start, out_queue):
    real_data = {"time": [], "position": [], "velocity": [], "torque": [], "voltage": [], "current": [], "temperature": [], "cmd_time": [], "cmd_pos": [], "cmd_vel": []}
    if args.test == "sine":
        asyncio.run(run_sine_test(
            kos=KOS(args.ip),
            actuator_id=args.actuator_id,
            amplitude=args.amp,
            freq=args.freq,
            duration=args.duration,
            kp=args.kp,
            kd=args.kd,
            ki=args.ki,
            sim_kp=args.sim_kp,
            sim_kv=args.sim_kv,
            acceleration=args.acceleration,
            max_torque=args.max_torque,
            torque_enabled=(not args.torque_off),
            update_rate=50.0,
            data_dict=real_data,
            start_time=global_start,
            is_real=True,
            start_pos=args.start_pos,
        ))
    elif args.test == "step":
        asyncio.run(run_step_test(
            kos=KOS(args.ip),
            actuator_id=args.actuator_id,
            step_size=args.step_size,
            step_hold_time=args.step_hold_time,
            step_count=args.step_count,
            kp=args.kp,
            kd=args.kd,
            ki=args.ki,
            sim_kp=args.sim_kp,
            sim_kv=args.sim_kv,
            acceleration=args.acceleration,
            max_torque=args.max_torque,
            torque_enabled=(not args.torque_off),
            vel_limit=400.0,
            data_dict=real_data,
            start_time=global_start,
            sample_rate=args.sample_rate,
            is_real=True,
            start_pos=args.start_pos,
        ))
    elif args.test == "chirp":
        asyncio.run(run_chirp_test(
            kos=KOS(args.ip),
            actuator_id=args.actuator_id,
            amplitude=args.chirp_amp,
            init_freq=args.chirp_init_freq,
            sweep_rate=args.chirp_sweep_rate,
            duration=args.chirp_duration,
            kp=args.kp,
            kd=args.kd,
            ki=args.ki,
            sim_kp=args.sim_kp,
            sim_kv=args.sim_kv,
            acceleration=args.acceleration,
            max_torque=args.max_torque,
            torque_enabled=(not args.torque_off),
            update_rate=50.0,
            data_dict=real_data,
            start_time=global_start,
            is_real=True,
            start_pos=args.start_pos,
        ))
    out_queue.put(real_data)



#############################
# MAIN (CLI + Orchestration)#
#############################
async def main():
    parser = argparse.ArgumentParser(
        description="ktune - CLI tool for actuator tests (sine or step) on both simulator and real robot."
    )
    parser.add_argument("--name", default="Zeroth01", help="Name For Plot titles")
    parser.add_argument("--sim_ip", default="127.0.0.1", help="Simulator KOS IP address (default=localhost)")
    parser.add_argument("--ip", default="192.168.42.1", help="Real robot KOS IP address (default=192.168.42.1)")
    parser.add_argument("--actuator-id", type=int, default=11, help="Actuator ID to test.")
    parser.add_argument("--test", choices=["step", "sine", "chirp"], help="Type of test to run.")
    parser.add_argument("--start-pos", type=float, default=0.0, help="Start position for tests (degrees)")
    # Chirp test parameters
    parser.add_argument("--chirp-amp", type=float, default=5.0, help="Chirp amplitude (degrees)")
    parser.add_argument("--chirp-init-freq", type=float, default=1.0, help="Chirp initial frequency (Hz)")
    parser.add_argument("--chirp-sweep-rate", type=float, default=0.5, help="Chirp sweep rate (Hz per second)")
    parser.add_argument("--chirp-duration", type=float, default=5.0, help="Chirp test duration (seconds)")

    # Sine test parameters
    parser.add_argument("--freq", type=float, default=1.0, help="Sine frequency (Hz)")
    parser.add_argument("--amp", type=float, default=5.0, help="Sine amplitude (degrees)")
    parser.add_argument("--duration", type=float, default=5.0, help="Sine test duration (seconds)")

    # Step test parameters
    parser.add_argument("--step-size", type=float, default=10.0, help="Step size (degrees)")
    parser.add_argument("--step-hold-time", type=float, default=3.0, help="Time to hold at step (seconds)")
    parser.add_argument("--step-count", type=int, default=2, help="Number of steps to take")

    # Simulation gains
    parser.add_argument("--sim-kp", type=float, default=24.0, help="Proportional gain")
    parser.add_argument("--sim-kv", type=float, default=0.75, help="Damping gain")

    # Actuator config
    parser.add_argument("--kp", type=float, default=20.0, help="Proportional gain")
    parser.add_argument("--kd", type=float, default=55.0, help="Derivative gain")
    parser.add_argument("--ki", type=float, default=0.01, help="Integral gain")
    parser.add_argument("--acceleration", type=float, default=0.0, help="Acceleration (deg/s^2)")
    parser.add_argument("--max-torque", type=float, default=100.0, help="Max torque")
    parser.add_argument("--torque-off", action="store_true", help="Disable torque for test?")

    # Data logging
    parser.add_argument("--no-log", action="store_true", help="Do not record/plot data")
    parser.add_argument("--log-duration-pad", type=float, default=2.0,
                        help="Pad (seconds) after motion ends to keep logging")
    parser.add_argument("--sample-rate", type=float, default=100.0, help="Data collection rate (Hz)")

    # Servo Enable/Disable
    parser.add_argument("--enable-servos", type=str, help="Comma delimited list of servo IDs to enable (e.g., 11,12,13)")
    parser.add_argument("--disable-servos", type=str, help="Comma delimited list of servo IDs to disable (e.g., 31,32,33)")
    parser.add_argument('--version', action='version', version=f'ktune v{__version__}')
    args = parser.parse_args()

    # Prepare separate data dictionaries for simulator and real robot.
    sim_data = {
        "time": [],
        "position": [],
        "velocity": [],
        "cmd_time": [],
        "cmd_pos": [],
        "cmd_vel": []
    }
    real_data = {
        "time": [],
        "position": [],
        "velocity": [],
        "cmd_time": [],
        "cmd_pos": [],
        "cmd_vel": []
    }
    now_str = datetime.now().strftime("%Y%m%d_%H%M%S")

    # Handle servo enable/disable separately from test execution
    if args.enable_servos is not None or args.disable_servos is not None:
        real_kos = KOS(args.ip)
        await configure_additional_servos(real_kos, args)
        await real_kos.close()

        sim_kos = KOS(args.sim_ip)
        await configure_additional_servos(sim_kos, args)
        await sim_kos.close()
        print("Servos configured")
        return
    elif not args.test:
        parser.print_help()
        exit(1)

    print(f"Connecting to Simulator at {args.sim_ip} and Real robot at {args.ip}...")

    print("Testing KOS-SIM connection performance...")
    sim_kos = KOS(args.sim_ip)
    sim_start = time.time()
    for _ in range(100):  # Test 100 samples
        await sim_kos.actuator.get_actuators_state([args.actuator_id])
    sim_end = time.time()
    sim_rate = 100 / (sim_end - sim_start)
    await sim_kos.close()

    print("Testing KOS-REAL connection performance...")
    real_kos = KOS(args.ip)
    real_start = time.time() 
    for _ in range(100):  # Test 100 samples
        await real_kos.actuator.get_actuators_state([args.actuator_id])
    real_end = time.time()
    real_rate = 100 / (real_end - real_start)
    await real_kos.close()

    await asyncio.sleep(1.0)
    print(f"Max KOS-SIM sampling rate: {sim_rate:.1f} Hz")
    print(f"Max KOS-REAL sampling rate: {real_rate:.1f} Hz")
    print(f"Required sampling rate: {args.sample_rate} Hz")

    if sim_rate < args.sample_rate or real_rate < args.sample_rate:
        print(f"\nERROR: Requested sampling rate ({args.sample_rate} Hz) exceeds maximum achievable rates")
        print("Try re-running kos-sim --no-render or reduce the sampling rate and try again")
        exit(1)
    

    global_start = time.time()
    sim_queue = Queue()
    real_queue = Queue()

    sim_proc = Process(target=run_sim_test, args=(args, global_start, sim_queue))
    real_proc = Process(target=run_real_test, args=(args, global_start, real_queue))

    sim_proc.start()
    real_proc.start()

    print("Waiting for data from simulator and real robot...")

    sim_data = sim_queue.get()
    real_data = real_queue.get()

    sim_proc.join(timeout=5)
    real_proc.join(timeout=5)

    print("Plotting data...")
        # Plotting both simulator and real data on the same plots.
    if not args.no_log:
        JOINT_NAMES = {
            11: "Left Shoulder Roll",
            12: "Left Shoulder Pitch", 
            13: "Left Elbow Roll",
            14: "Left Gripper",
            21: "Right Shoulder Roll",
            22: "Right Shoulder Pitch",
            23: "Right Elbow Roll", 
            24: "Right Gripper",
            31: "Left Hip Yaw",
            32: "Left Hip Roll",
            33: "Left Hip Pitch",
            34: "Left Knee Pitch",
            35: "Left Ankle Pitch",
            41: "Right Hip Yaw",
            42: "Right Hip Roll",
            43: "Right Hip Pitch",
            44: "Right Knee Pitch",
            45: "Right Ankle Pitch"
        }

        os.makedirs("plots", exist_ok=True)
        os.makedirs("raw_data", exist_ok=True)

        # Get joint name and create sanitized version for directory name
        joint_name = JOINT_NAMES.get(args.actuator_id, f"id_{args.actuator_id}")
        dir_name = joint_name.lower().replace(" ", "_")

         # Create subdirectories
        plot_dir = os.path.join("plots", dir_name)
        data_dir = os.path.join("raw_data", dir_name)
        os.makedirs(plot_dir, exist_ok=True)
        os.makedirs(data_dir, exist_ok=True)

        test_joint = JOINT_NAMES.get(args.actuator_id, "")

        # Build header for JSON file.
        header = {
            "test_type": args.test,
            "actuator_id": args.actuator_id,
            "joint_name": test_joint,
            "timestamp": now_str,
            "robot_name": args.name,
            "start_position": args.start_pos,
            "gains": {
                "sim": {"kp": args.sim_kp, "kv": args.sim_kv},
                "real": {"kp": args.kp, "kd": args.kd, "ki": args.ki}
            },
            "acceleration": args.acceleration
        }
        # Build a title string based on the test type.
        if args.test == "chirp":
            header.update({
                "initial_frequency": args.chirp_init_freq,
                "sweep_rate": args.chirp_sweep_rate,
                "amplitude": args.chirp_amp,
                "duration": args.chirp_duration
            })
            title_str = (f"{args.name} -- Chirp Test -- ID: {args.actuator_id} {test_joint}\n"
                        f"Center: {args.start_pos}°, Init Freq: {args.chirp_init_freq} Hz, "
                        f"Sweep Rate: {args.chirp_sweep_rate} Hz/s, Amp: {args.chirp_amp}°, Duration: {args.chirp_duration}s\n"
                        f"Update Rate: 50 Hz\n"
                        f"Sim Kp: {args.sim_kp} Kv: {args.sim_kv} | Real Kp: {args.kp} Kd: {args.kd} Ki: {args.ki}\n"
                        f"Acceleration: {args.acceleration:.0f} deg/s²")
        elif args.test == "sine":
            header.update({
                "frequency": args.freq,
                "amplitude": args.amp,
                "duration": args.duration,
                "command_rate": 50,  # Hz
                "sample_rate": args.sample_rate
            })
            title_str = (f"{args.name} -- Sine Wave Test -- ID: {args.actuator_id} {test_joint}\n"
                        f"Center: {args.start_pos}°, Freq: {args.freq} Hz, Amp: {args.amp}°, "
                        f"Sample/Ctrl Rate: {args.sample_rate} Hz\n"
                        f"Sim Kp: {args.sim_kp} Kv: {args.sim_kv} | Real Kp: {args.kp} Kd: {args.kd} Ki: {args.ki}\n"
                        f"Acceleration: {args.acceleration:.0f} deg/s²")
        elif args.test == "step":
            # Construct the step sequence as used in the test:
            # Initial hold at 0°, then for each cycle: step up to step_size then step down to 0°
            vel = args.vel_limit if hasattr(args, "vel_limit") else 200.0
            steps_list = [(0.0, vel, args.step_hold_time)]
            for _ in range(args.step_count):
                steps_list.append((args.step_size, vel, args.step_hold_time))
                steps_list.append((0.0, vel, args.step_hold_time))
            
            # Compute overshoots using the collected time and position data.
            overshoots_sim = compute_step_overshoots(np.array(sim_data["time"]), np.array(sim_data["position"]), steps_list, window_duration=1.0)
            overshoots_real = compute_step_overshoots(np.array(real_data["time"]), np.array(real_data["position"]), steps_list, window_duration=1.0)
            max_overshoot_sim = max(overshoots_sim) if len(overshoots_sim) > 0 else 0.0
            max_overshoot_real = max(overshoots_real) if len(overshoots_real) > 0 else 0.0

            header.update({
                "step_size": args.step_size,
                "step_hold_time": args.step_hold_time,
                "step_count": args.step_count,
                "velocity_limit": vel,
                "max_overshoot": {
                    "sim": max(overshoots_sim) if overshoots_sim else 0.0,
                    "real": max(overshoots_real) if overshoots_real else 0.0
                }
            })
            title_str = (
                f"{args.name} -- Step Test -- ID: {args.actuator_id} {test_joint}\n"
                f"Center: {args.start_pos}°, Step Size: ±{args.step_size}°, Hold: {args.step_hold_time}s, Count: {args.step_count}\n"
                f"Update Rate: {args.sample_rate} Hz\n"
                f"Sim Kp: {args.sim_kp} Kv: {args.sim_kv} | Real Kp: {args.kp} Kd: {args.kd} Ki: {args.ki}\n"
                f"Overshoot - Sim: {max_overshoot_sim:.1f}%  Real: {max_overshoot_real:.1f}%\n"
                f"Acceleration: {args.acceleration:.0f} deg/s²"
            )
        else:
            title_str = f"{args.test.capitalize()} Test - Actuator {args.actuator_id}"

        # Save raw data to JSON files.
        sim_output = {
            "header": header,
            "data": sim_data
        }
        real_output = {
            "header": header,
            "data": real_data
        }
        json_base_path = os.path.join(data_dir, f"{now_str}_{args.test}")
        with open(f"{json_base_path}_sim.json", "w") as f:
            json.dump(sim_output, f, indent=2)
        with open(f"{json_base_path}_real.json", "w") as f:
            json.dump(real_output, f, indent=2)
        print(f"Saved raw data to {json_base_path}_sim.json and {json_base_path}_real.json")


        # Plotting both simulator and real data on the same plots.
        fig, axs = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
        fig.suptitle(title_str, fontsize=16)
        axs[0, 0].plot(sim_data["cmd_time"], sim_data["cmd_pos"], '--', color='black', linewidth=1, label='Sim Command Pos')
        axs[0, 0].plot(sim_data["time"], sim_data["position"], 'o-', color='blue', markersize=2, label='Sim Actual Pos')
        axs[0, 0].set_title("Sim - Position")
        axs[0, 0].set_ylabel("Position (deg)")
        axs[0, 0].legend()
        axs[0, 0].grid(True)

        if args.test == "sine":
            axs[1, 0].plot(sim_data["cmd_time"], sim_data["cmd_vel"], '--', color='black', linewidth=1, label='Sim Command Vel')
        axs[1, 0].plot(sim_data["time"], sim_data["velocity"], 'o-', color='blue', markersize=2, label='Sim Actual Vel')
        axs[1, 0].set_title("Sim - Velocity")
        axs[1, 0].set_xlabel("Time (s)")
        axs[1, 0].set_ylabel("Velocity (deg/s)")
        axs[1, 0].legend()
        axs[1, 0].grid(True)

        # Real Robot subplots (right column)
        axs[0, 1].plot(real_data["cmd_time"], real_data["cmd_pos"], '--', color='black', linewidth=1, label='Real Command Pos')
        axs[0, 1].plot(real_data["time"], real_data["position"], 's-', color='red', markersize=2, linewidth=1, label='Real Actual Pos')
        axs[0, 1].set_title("Real - Position")
        axs[0, 1].set_ylabel("Position (deg)")
        axs[0, 1].legend()
        axs[0, 1].grid(True)

        if args.test == "sine":
            axs[1, 1].plot(real_data["cmd_time"], real_data["cmd_vel"], '--', color='black', linewidth=1, label='Real Command Vel')
        axs[1, 1].plot(real_data["time"], real_data["velocity"], 's-', color='red', markersize=2, linewidth=1,label='Real Actual Vel')
        axs[1, 1].set_title("Real - Velocity")
        axs[1, 1].set_xlabel("Time (s)")
        axs[1, 1].set_ylabel("Velocity (deg/s)")
        axs[1, 1].legend()
        axs[1, 1].grid(True)

        plt.figtext(0.5, 0.02, f"ktune v{__version__}", ha='center', va='center', fontsize=12)
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        png_path = os.path.join(plot_dir, f"{now_str}_{args.test}.png")
        plt.savefig(png_path)
        print(f"Saving plot data to {plot_dir}/{args.test}_test_{now_str}.png")
        plt.show()
        plt.close()


    print("Test complete.")

def cli():
    """Entry point for the command line interface"""
    asyncio.run(main())

if __name__ == "__main__":
    asyncio.run(main())
