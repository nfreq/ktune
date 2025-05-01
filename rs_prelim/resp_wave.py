import asyncio
import time
import random
import pykos
import numpy as np
import json
from datetime import datetime
import os

import colorlogging
import logging

logger = logging.getLogger(__name__)

colorlogging.configure()

async def run_wave_test(
    actuator_ids, 
    kp, kd, max_torque, min_pos, max_pos,
    start_pos, amplitude, frequency, phase_offset, position_offset, duration):

    kos = pykos.KOS("0.0.0.0")
    
    # Initialize data storage
    data = []
    sample_rate = 100  # Hz
    sample_interval = 1.0 / sample_rate

    for actuator_id in actuator_ids:
        await kos.actuator.configure_actuator(
            actuator_id=actuator_id,
            kp=kp,
            kd=kd,
            max_torque=max_torque,
            torque_enabled=True,
        )

    logger.info(f"Moving to start position: {start_pos} degrees")
    for actuator_id in actuator_ids:
        commands = [
            {
                "actuator_id": actuator_id,
                "position": start_pos,
            }
        ]

        await kos.actuator.command_actuators(commands)

    await asyncio.sleep(2.0)

    test_start_time = time.time()  

    t = np.arange(0, duration, 1/sample_rate)  

    for current_time in t:
        angle = 2 * np.pi * frequency * current_time + np.deg2rad(phase_offset)
        position = start_pos + amplitude * np.sin(angle) + position_offset

        commanded_position = position
        if commanded_position > max_pos:
            commanded_position = max_pos
            logger.warning(f"Commanded position {commanded_position} is greater than max position {max_pos}")
        if commanded_position < min_pos:
            commanded_position = min_pos
            logger.warning(f"Commanded position {commanded_position} is less than min position {min_pos}")
        
        logger.info(f"Position: {position} degrees")
        
        commands = [
            {
                'actuator_id': actuator_id,
                'position': position,
            }
            for actuator_id in actuator_ids
        ]
        await kos.actuator.command_actuators(commands)
        

        # Get and store timestamp and position
        timestamp = time.time() - test_start_time
        state = await kos.actuator.get_actuators_state([actuator_ids[0]])
        position = state.states[0].position
        data.append((timestamp, position, commanded_position))
        
        # ecise timing to maintain 100Hz
        next_time = test_start_time + current_time
        sleep_time = max(0, next_time - time.time())
        logger.info(f"Sleeping for {sleep_time} seconds")
        await asyncio.sleep(sleep_time)

    # Save collected data to JSON
    fldr_name =  datetime.now().strftime("%Y%m%d")
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{fldr_name}/{amplitude}deg_resp_wave_{timestamp_str}.json"
    
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
            "amplitude": amplitude,
            "frequency": frequency,
            "phase_offset": phase_offset,
            "position_offset": position_offset,
            "actuator_ids": actuator_ids
        },
        "data": json_data
    }
    
    os.makedirs(fldr_name, exist_ok=True)
    # Write to JSON file
    with open(filename, 'w') as jsonfile:
        json.dump(output, jsonfile, indent=2)
    
    logger.info(f"Data saved to {filename}")


if __name__ == "__main__":
    TEST_CONFIGS_03 = {
        "kp": 85.0, 
        "kd": 5.0, 
        "max_torque": 60.0, 
        "min_pos": -30.0,
        "max_pos": 30.0,

        "start_pos": 0.0,       # degrees
        "amplitude": 10.0,       # degrees
        "frequency": 0.5,        # Hz
        "phase_offset": 0.0,            # degrees
        "position_offset": 20.0,       # degrees
        "duration": 10.0,        # seconds
    }

    asyncio.run(run_wave_test([31], **TEST_CONFIGS_03))

