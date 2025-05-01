import asyncio
import time
import random
import pykos
import numpy as np
import json
from datetime import datetime
import os
import argparse
import colorlogging
import logging

logger = logging.getLogger(__name__)

colorlogging.configure()


async def sample_loop(kos, actuator_id, data, test_start, duration, commanded_position):
    sampling_rate = 100
    end = time.monotonic() + duration
    while time.monotonic() < end:
        t0 = time.monotonic()
        state = await kos.actuator.get_actuators_state([actuator_id])
        data.append((t0 - test_start, state.states[0].position, commanded_position))
        await asyncio.sleep(max(0, 1.0/sampling_rate - (time.monotonic() - t0)))


async def run_step_test(
    actuator_ids, 
    kp, kd, max_torque, min_pos, max_pos,
    step_size, step_hold_time, 
    step_count, start_pos, 
    single_mode, random_mode, seed):

    kos = pykos.KOS("0.0.0.0")
    
    # Initialize data storage
    data = []
    sample_rate = 100  # Hz
    sample_interval = 1.0 / sample_rate
        # Current commanded position
    commanded_position = start_pos

    await kos.actuator.configure_actuator(
        actuator_id=actuator_ids[0],
        kp=kp,
        kd=kd,
        max_torque=max_torque,
        torque_enabled=True,
    )

    logger.info(f"Moving to start position: {start_pos} degrees")
    commands = [
        {
            "actuator_id": actuator_ids[0],
            "position": start_pos,
        }
    ]

    await kos.actuator.command_actuators(commands)
    await asyncio.sleep(2.0)

    
    test_start_time = time.monotonic()


    if random_mode:
        random.seed(seed)

        steps = []
        current_pos = start_pos
        
        for _ in range(step_count):
            # Generate random step size (magnitude only)

            valid_directions = []
            while not valid_directions:
                step_size = random.uniform(1, 10)
                
                # Check if we can move positive
                if (current_pos + step_size) <= max_pos:
                    valid_directions.append(1)
                
                # Check if we can move negative
                if (current_pos - step_size) >= min_pos:
                    valid_directions.append(-1)
                
            direction = random.choice(valid_directions)
            
            # Calculate next position
            target_pos = current_pos + (step_size * direction)
            steps.append(target_pos)
            current_pos = target_pos


        for i, target_pos in enumerate(steps, 1):
            sampler = asyncio.create_task(sample_loop(kos, actuator_ids[0], data, test_start_time, step_hold_time, target_pos))

            commands = [
                {
                    'actuator_id': actuator_ids[0],
                    'position': target_pos,
                }
            ]
            await kos.actuator.command_actuators(commands)
            await asyncio.sleep(step_hold_time)
            await sampler

    elif single_mode:
        duration = step_hold_time
        target_pos = start_pos + step_size
        sampler = asyncio.create_task(sample_loop(kos, actuator_ids[0], data, test_start_time, duration, target_pos))
        commands = [
            {
                'actuator_id': actuator_id,
                'position': target_pos,
            }
            for actuator_id in actuator_ids
        ]
        await kos.actuator.command_actuators(commands)
        await asyncio.sleep(step_hold_time)
        await sampler


    else:
        logger.info(f"Moving {step_count} steps of {step_size} degrees each")
        for step in range(step_count):
            target_pos = start_pos + step_size
            commanded_position = target_pos
            if commanded_position > max_pos:
                commanded_position = max_pos
                logger.warning(f"Commanded position {commanded_position} is greater than max position {max_pos}")
            if commanded_position < min_pos:
                commanded_position = min_pos
                logger.warning(f"Commanded position {commanded_position} is less than min position {min_pos}")
            
            duration = step_hold_time
            sampler = asyncio.create_task(sample_loop(kos, actuator_ids[0], data, test_start_time, duration, target_pos))

            logger.info(target_pos)
            logger.info(f"\nStep {step + 1}/{step_count} UP to {target_pos}°")
            commands = [
                {
                    'actuator_id': actuator_id,
                    'position': target_pos,
                }
                for actuator_id in actuator_ids
            ]
            await kos.actuator.command_actuators(commands)

            await sampler


            sampler = asyncio.create_task(sample_loop(kos, actuator_ids[0], data, test_start_time, duration, start_pos))

            logger.info(f"Step {step + 1}/{step_count} DOWN to {start_pos}°")
            commanded_position = start_pos
            commands = [
                {
                    'actuator_id': actuator_id,
                    'position': start_pos,
                }
                for actuator_id in actuator_ids
            ]
            await kos.actuator.command_actuators(commands)

            await sampler
            

    # Save collected data to JSON
    fldr_name =  datetime.now().strftime("%Y%m%d")
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{fldr_name}/{step_hold_time}s_resp_step_{timestamp_str}.json"
    
    # Convert data to list of dictionaries for JSON serialization
    json_data = []
    for entry in data:
        json_data.append({
            "time_since_start": entry[0],
            "position": entry[1],
            "commanded_position": entry[2]
        })
    
    # Create JSON structure with config as header and data points
    output = {
        "config": {
            "kp": kp,
            "kd": kd,
            "max_torque": max_torque,
            "step_size": step_size,
            "step_hold_time": step_hold_time,
            "step_count": step_count,
            "start_pos": start_pos,
            "actuator_ids": actuator_ids
        },
        "data": json_data
    }
    
    # Write to JSON file
    os.makedirs(fldr_name, exist_ok=True)
    with open(filename, 'w') as jsonfile:
        json.dump(output, jsonfile, indent=2)
    
    logger.info(f"Data saved to {filename}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--randomize", action="store_true")
    parser.add_argument("--single", action="store_true")
    args = parser.parse_args()

    if args.single and args.randomize:
        logger.warning("Cannot randomize and single step at the same time")
        exit(1)
    elif args.randomize:
        logger.warning("Confirm you want to randomize, and you have set the min and max pos accordingly")
        input("Press Enter to continue")
    

    
    TEST_CONFIGS_03 = {
        "kp": 85.0, 
        "kd": 5.0, 
        "max_torque": 60.0,
        "min_pos": -30.0,
        "max_pos": 30.0,

        "step_hold_time": 0.50, # seconds
        "step_count": 100,  # 1000
        "start_pos": 0.0,       # degrees

        "step_size": 14.0,       # degrees

        "single_mode": args.single,
        "random_mode": args.randomize,
        "seed": 43, 
    }

    asyncio.run(run_step_test([33], **TEST_CONFIGS_03))

