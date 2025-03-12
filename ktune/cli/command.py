# ktune/ktune/cli/commands.py
import os
import json
from datetime import datetime
import click
import yaml
from typing import Optional, Dict
from ktune.config.validation import ConfigValidator
from ktune.core.tune import Tune
from ktune.core.sysid.testbed.pendulum import PendulumBench, PendulumConfig
from ktune.core.utils import metrics
from pykos import KOS
import numpy as np

@click.group()
def cli():
    """KTune - Motor tuning and system identification toolkit"""
    pass

@cli.group()
@click.option('--config', type=click.Path(exists=True), help='Path to config file')
@click.option('--name', default="Zeroth01", help='Name for plot titles')
@click.option('--sim-ip', default="127.0.0.1", help='Simulator KOS IP address')
@click.option('--real-ip', default="192.168.42.1", help='Real robot KOS IP address')
@click.option('--actuator-id', type=int, default=11, help='Actuator ID to test')
@click.option('--start-pos', type=float, default=0.0, help='Start position (degrees)')
# Actuator config
@click.option('--kp', type=float, default=20.0, help='Proportional gain')
@click.option('--kd', type=float, default=55.0, help='Derivative gain')
@click.option('--ki', type=float, default=0.01, help='Integral gain')
@click.option('--acceleration', type=float, default=0.0, help='Acceleration (deg/s^2)')
@click.option('--max-torque', type=float, default=100.0, help='Max torque')
@click.option('--torque-off', is_flag=True, help='Disable torque for test?')
# Simulation gains
@click.option('--sim-kp', type=float, default=24.0, help='Simulation proportional gain')
@click.option('--sim-kv', type=float, default=0.75, help='Simulation damping gain')
# Data logging
@click.option('--no-log', is_flag=True, help='Do not record/plot data')
@click.option('--log-duration-pad', type=float, default=2.0,
              help='Pad (seconds) after motion ends to keep logging')
@click.option('--sample-rate', type=float, default=100.0, help='Data collection rate (Hz)')
# Servo Enable/Disable
@click.option('--enable-servos', help='Comma delimited list of servo IDs to enable (e.g., 11,12,13)')
@click.option('--disable-servos', help='Comma delimited list of servo IDs to disable (e.g., 31,32,33)')
@click.pass_context
def tune(ctx, **kwargs):
    """Run tuning tests"""
    # Store configuration in context for subcommands
    ctx.ensure_object(dict)
    
    # Initialize configuration
    cfg = {'tune': {}}

    # Load config file if provided
    if kwargs.get('config'):
        try:
            with open(kwargs['config']) as f:
                cfg = yaml.safe_load(f)
        except (yaml.YAMLError, IOError) as e:
            click.echo(f"Error loading config file: {e}", err=True)
            raise click.Abort()

    # Process servo lists
    if kwargs.get('enable_servos'):
        kwargs['enable_servos'] = [
            int(x.strip()) for x in kwargs['enable_servos'].split(',')
        ]
    if kwargs.get('disable_servos'):
        kwargs['disable_servos'] = [
            int(x.strip()) for x in kwargs['disable_servos'].split(',')
        ]

    # Store processed kwargs in context
    ctx.obj['config'] = cfg
    ctx.obj['cli_args'] = {k: v for k, v in kwargs.items() if v is not None}

@tune.command()
@click.option('--freq', type=float, default=1.0, help='Sine frequency (Hz)')
@click.option('--amp', type=float, default=5.0, help='Sine amplitude (degrees)')
@click.option('--duration', type=float, default=5.0, help='Duration (seconds)')
@click.pass_context
def sine(ctx, **kwargs):
    """Run sine wave test"""
    config = ctx.obj['config']
    cli_args = ctx.obj['cli_args']

    # Add sine-specific parameters
    cli_args['test'] = 'sine'
    cli_args['freq'] = kwargs['freq']
    cli_args['amp'] = kwargs['amp']
    cli_args['duration'] = kwargs['duration']

    # Update config with CLI arguments
    config.setdefault('tune', {}).update(cli_args)

    # Validate and run
    _validate_and_run(config)

@tune.command()
@click.option('--size', type=float, default=10.0, help='Step size (degrees)')
@click.option('--hold-time', type=float, default=3.0, help='Hold time (seconds)')
@click.option('--count', type=int, default=2, help='Number of steps')
@click.pass_context
def step(ctx, **kwargs):
    """Run step response test"""
    config = ctx.obj['config']
    cli_args = ctx.obj['cli_args']

    # Add step-specific parameters
    cli_args['test'] = 'step'
    cli_args['step_size'] = kwargs['size']
    cli_args['step_hold_time'] = kwargs['hold_time']
    cli_args['step_count'] = kwargs['count']

    # Update config with CLI arguments
    config.setdefault('tune', {}).update(cli_args)

    # Validate and run
    _validate_and_run(config)

@tune.command()
@click.option('--amp', type=float, default=5.0, help='Chirp amplitude (degrees)')
@click.option('--init-freq', type=float, default=1.0, help='Initial frequency (Hz)')
@click.option('--sweep-rate', type=float, default=0.5, help='Sweep rate (Hz/s)')
@click.option('--duration', type=float, default=5.0, help='Duration (seconds)')
@click.pass_context
def chirp(ctx, **kwargs):
    """Run chirp test"""
    config = ctx.obj['config']
    cli_args = ctx.obj['cli_args']

    # Add chirp-specific parameters
    cli_args['test'] = 'chirp'
    cli_args['chirp_amp'] = kwargs['amp']
    cli_args['chirp_init_freq'] = kwargs['init_freq']
    cli_args['chirp_sweep_rate'] = kwargs['sweep_rate']
    cli_args['chirp_duration'] = kwargs['duration']

    # Update config with CLI arguments
    config.setdefault('tune', {}).update(cli_args)

    # Validate and run
    _validate_and_run(config)

def _validate_and_run(config: Dict):
    """Helper function to validate config and run tune"""
    validator = ConfigValidator()
    try:
        # Apply defaults to missing values
        config = validator.apply_defaults(config)
        
        # Validate all sections
        validator.validate_all(config)
    except ValueError as e:
        click.echo(f"Configuration error: {e}", err=True)
        raise click.Abort()

    # Initialize and run tuner
    ktune = Tune(config)
    ktune.run_test(config['tune'].get('test'))

@cli.command()
def version():
    """Show the version of KTune"""
    import ktune
    click.echo(f"ktune v{ktune.__version__}")

    pass


@cli.group()
def sysid():
    """Run system identification experiments"""
    pass

@sysid.command()
@click.option('--config', type=click.Path(exists=True), help='Path to config file')
@click.option('--ip', default="192.168.42.1", help='KOS IP address')
@click.option('--actuator-id', type=int, default=11, help='Actuator ID to test')
# Motor parameters
@click.option('--motor-name', default="sts3215", help='Motor model name')
@click.option('--winding-resistance', type=float, default=2.1, help='Motor winding resistance (ohms)')
@click.option('--torque-constant', type=float, default=0.0955, help='Motor torque constant (Nm/A)')
# Control parameters
@click.option('--kp', type=float, default=32.0, help='Position gain')
@click.option('--error-gain', type=float, default=1.0, help='Error gain for system ID')
# Pendulum parameters
@click.option('--mass', type=float, required=True, help='Pendulum mass (kg)')
@click.option('--length', type=float, required=True, help='Pendulum length (m)')
# Test configuration
@click.option('--trajectory', type=str, required=True,
              help='Trajectory type: lift_and_drop, sin_time_square, up_and_down, sin_sin, brutal, nothing')
@click.option('--sample-rate', type=float, default=100.0, help='Data collection rate (Hz)')
@click.pass_context
def pendulum(ctx, **kwargs):
    """Run pendulum system identification experiment"""
    # Store configuration in context
    ctx.ensure_object(dict)
    
    # Initialize configuration
    cfg = {'sysid': {}}

    # Load config file if provided
    if kwargs.get('config'):
        try:
            with open(kwargs['config']) as f:
                cfg = yaml.safe_load(f)
        except (yaml.YAMLError, IOError) as e:
            click.echo(f"Error loading config file: {e}", err=True)
            raise click.Abort()

    # Update config with CLI arguments
    cfg.setdefault('sysid', {}).update(kwargs)

    # Validate and run
    _validate_and_run_sysid(cfg)

def _validate_and_run_sysid(config: Dict):
    """Helper function to validate config and run sysid experiment"""
    try:
        cfg = config['sysid']

        kos = KOS(cfg['ip'])
        
        # Create pendulum config
        pendulum_config = PendulumConfig(
            motor=cfg['motor_name'],
            actuator_id=cfg['actuator_id'],
            ip=cfg['ip'],  # Make sure we pass IP from CLI args
            mass=cfg['mass'],
            length=cfg['length'],
            kp=cfg['kp'],
            max_torque=cfg.get('max_torque', 100.0),
            acceleration=0.0,  # Fixed for pendulum experiments
            sample_rate=cfg.get('sample_rate', 100.0),
            vin=cfg.get('vin', 15.0),
            offset=cfg.get('offset', 0.0)
        )

        # Initialize bench
        bench = PendulumBench(pendulum_config)
        
        # Run experiment and save data
        data = bench.run_experiment(cfg['trajectory'])  # Let PendulumBench handle async
        
        # Add motor parameters to data
        data['motor_params'] = {
            'name': cfg['motor_name'],
            'winding_resistance': cfg['winding_resistance'],
            'torque_constant': cfg['torque_constant']
        }
        
        # Save data
        os.makedirs('logs', exist_ok=True)
        timestamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
        filename = f"logs/sysid_{cfg['motor_name']}_{cfg['trajectory']}_{timestamp}.json"
        
        with open(filename, 'w') as f:
            json.dump(data, f)
            
        click.echo(f"Data saved to {filename}")

    except Exception as e:
        import traceback
        print(f"Exception details:\n{traceback.format_exc()}")
        click.echo(f"Error running experiment: {e}", err=True)
        raise click.Abort()

if __name__ == '__main__':
    cli()