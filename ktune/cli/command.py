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

def create_mode_command(mode: str):
    """Create a command decorator with mode-specific options"""
    def decorator(f):
        # Add common options first
        f = add_common_options(f)
        
        # Add mode-specific options
        if mode == 'compare':
            f = click.option('--sim-ip', default="127.0.0.1", help='Simulator KOS IP address')(f)
            f = click.option('--real-ip', default="192.168.42.1", help='Real robot KOS IP address')(f)
            f = click.option('--sim-kp', type=float, default=20.0, help='Simulation proportional gain')(f)
            f = click.option('--sim-kd', type=float, default=5.0, help='Simulation damping gain')(f)
            f = click.option('--stream-delay', type=float, default=0.0, help='Simulation stream delay (seconds)')(f)
        elif mode == 'real':
            f = click.option('--real-ip', default="192.168.42.1", help='Real robot KOS IP address')(f)
        elif mode == 'sim':
            f = click.option('--sim-ip', default="127.0.0.1", help='Simulator KOS IP address')(f)
            f = click.option('--sim-kp', type=float, default=20.0, help='Simulation proportional gain')(f)
            f = click.option('--sim-kd', type=float, default=5.0, help='Simulation damping gain')(f)
            f = click.option('--stream-delay', type=float, default=0.0, help='Simulation stream delay (seconds)')(f)
        return f
    return decorator

def create_test_command(test_type: str):
    """Create a command decorator with test-specific options"""
    def decorator(f):
        if test_type == 'sine':
            f = click.option('--freq', type=float, default=1.0, help='Sine frequency (Hz)')(f)
            f = click.option('--amp', type=float, default=5.0, help='Sine amplitude (degrees)')(f)
            f = click.option('--duration', type=float, default=5.0, help='Duration (seconds)')(f)
        elif test_type == 'step':
            f = click.option('--step-size', type=float, default=10.0, help='Step size (degrees)')(f)
            f = click.option('--step-hold-time', type=float, default=3.0, help='Hold time (seconds)')(f)
            f = click.option('--step-count', type=int, default=2, help='Number of steps')(f)
        elif test_type == 'chirp':
            f = click.option('--chirp-amp', type=float, default=5.0, help='Chirp amplitude (degrees)')(f)
            f = click.option('--chirp-init-freq', type=float, default=1.0, help='Initial frequency (Hz)')(f)
            f = click.option('--chirp-sweep-rate', type=float, default=0.5, help='Sweep rate (Hz/s)')(f)
            f = click.option('--chirp-duration', type=float, default=5.0, help='Duration (seconds)')(f)
        return f
    return decorator

def add_common_options(command):
    """Add common options to a command group"""
    options = [
        click.option('--config', type=click.Path(exists=True), help='Path to config file'),
        click.option('--name', default="NoName", help='Name for plot titles'),
        click.option('--actuator-id', type=int, default=11, help='Actuator ID to test'),
        click.option('--start-pos', type=float, default=0.0, help='Start position (degrees)'),
        click.option('--kp', type=float, default=20.0, help='Proportional gain'),
        click.option('--kd', type=float, default=5.0, help='Derivative gain'),
        click.option('--ki', type=float, default=0.0, help='Integral gain'),
        click.option('--acceleration', type=float, default=0.0, help='Acceleration (deg/s^2)'),
        click.option('--max-torque', type=float, default=100.0, help='Max torque'),
        click.option('--torque-off', is_flag=True, help='Disable torque for test?'),
        click.option('--no-log', is_flag=True, help='Do not record/plot data'),
        click.option('--log-duration-pad', type=float, default=2.0,
                    help='Pad (seconds) after motion ends to keep logging'),
        click.option('--sample-rate', type=float, default=100.0, help='Data collection rate (Hz)'),
        click.option('--enable-servos', help='Comma delimited list of servo IDs to enable'),
        click.option('--disable-servos', help='Comma delimited list of servo IDs to disable')
    ]
    for option in options:
        command = option(command)
    return command

def _handle_common_setup(ctx, kwargs, mode):
    """Common setup for all modes"""
    ctx.ensure_object(dict)
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
        kwargs['enable_servos'] = [int(x.strip()) for x in kwargs['enable_servos'].split(',')]
    if kwargs.get('disable_servos'):
        kwargs['disable_servos'] = [int(x.strip()) for x in kwargs['disable_servos'].split(',')]

    # Add mode to config
    kwargs['mode'] = mode
    
    # Store processed kwargs in context
    ctx.obj['config'] = cfg
    ctx.obj['cli_args'] = {k: v for k, v in kwargs.items() if v is not None}

def handle_test(ctx, kwargs, mode: str, test_type: str):
    """Common handler for all test commands"""
    _handle_common_setup(ctx, kwargs, mode=mode)
    ctx.obj['config'].setdefault('tune', {}).update(ctx.obj['cli_args'])
    ctx.obj['config']['tune']['test'] = test_type
    _validate_and_run(ctx.obj['config'])

# Define the mode groups
# ... existing code ...

@cli.group()
def compare():
    """Run comparison tests between real and simulated systems.
    
    Available tests:
    - sine: Run sinusoidal motion tests
    - step: Run step response tests
    - chirp: Run frequency sweep tests"""
    pass

@cli.group()
def real():
    """Run tests on real hardware only.
    
    Available tests:
    - sine: Run sinusoidal motion tests
    - step: Run step response tests
    - chirp: Run frequency sweep tests"""
    pass

@cli.group()
def sim():
    """Run tests on simulator only.
    
    Available tests:
    - sine: Run sinusoidal motion tests
    - step: Run step response tests
    - chirp: Run frequency sweep tests"""
    pass

# Add missing commands for each mode
# Real mode commands
@real.command(name='sine')
@create_mode_command('real')
@create_test_command('sine')
@click.pass_context
def real_sine(ctx, **kwargs):
    """Run sine wave test on real hardware.
    
    Parameters:
    --freq: Sine frequency in Hz
    --amp: Sine amplitude in degrees
    --duration: Test duration in seconds"""
    handle_test(ctx, kwargs, 'real', 'sine')

@real.command(name='step')
@create_mode_command('real')
@create_test_command('step')
@click.pass_context
def real_step(ctx, **kwargs):
    """Run step response test on real hardware.
    
    Parameters:
    --step-size: Size of step in degrees
    --step-hold-time: Hold time at each step in seconds
    --step-count: Number of steps to perform"""
    handle_test(ctx, kwargs, 'real', 'step')

@real.command(name='chirp')
@create_mode_command('real')
@create_test_command('chirp')
@click.pass_context
def real_chirp(ctx, **kwargs):
    """Run chirp (frequency sweep) test on real hardware.
    
    Parameters:
    --chirp-amp: Amplitude in degrees
    --chirp-init-freq: Initial frequency in Hz
    --chirp-sweep-rate: Rate of frequency increase in Hz/s
    --chirp-duration: Test duration in seconds"""
    handle_test(ctx, kwargs, 'real', 'chirp')

# Sim mode commands
@sim.command(name='sine')
@create_mode_command('sim')
@create_test_command('sine')
@click.pass_context
def sim_sine(ctx, **kwargs):
    """Run sine wave test in simulator.
    
    Parameters:
    --freq: Sine frequency in Hz
    --amp: Sine amplitude in degrees
    --duration: Test duration in seconds"""
    handle_test(ctx, kwargs, 'sim', 'sine')

@sim.command(name='step')
@create_mode_command('sim')
@create_test_command('step')
@click.pass_context
def sim_step(ctx, **kwargs):
    """Run step response test in simulator.
    
    Parameters:
    --step-size: Size of step in degrees
    --step-hold-time: Hold time at each step in seconds
    --step-count: Number of steps to perform"""
    handle_test(ctx, kwargs, 'sim', 'step')

@sim.command(name='chirp')
@create_mode_command('sim')
@create_test_command('chirp')
@click.pass_context
def sim_chirp(ctx, **kwargs):
    """Run chirp (frequency sweep) test in simulator.
    
    Parameters:
    --chirp-amp: Amplitude in degrees
    --chirp-init-freq: Initial frequency in Hz
    --chirp-sweep-rate: Rate of frequency increase in Hz/s
    --chirp-duration: Test duration in seconds"""
    handle_test(ctx, kwargs, 'sim', 'chirp')

# Compare mode commands
@compare.command(name='sine')
@create_mode_command('compare')
@create_test_command('sine')
@click.pass_context
def compare_sine(ctx, **kwargs):
    """Run sine wave comparison test between real and simulated systems.
    
    Parameters:
    --freq: Sine frequency in Hz
    --amp: Sine amplitude in degrees
    --duration: Test duration in seconds"""
    handle_test(ctx, kwargs, 'compare', 'sine')

@compare.command(name='step')
@create_mode_command('compare')
@create_test_command('step')
@click.pass_context
def compare_step(ctx, **kwargs):
    """Run step response comparison test between real and simulated systems.
    
    Parameters:
    --step-size: Size of step in degrees
    --step-hold-time: Hold time at each step in seconds
    --step-count: Number of steps to perform"""
    handle_test(ctx, kwargs, 'compare', 'step')

@compare.command(name='chirp')
@create_mode_command('compare')
@create_test_command('chirp')
@click.pass_context
def compare_chirp(ctx, **kwargs):
    """Run chirp (frequency sweep) comparison test between real and simulated systems.
    
    Parameters:
    --chirp-amp: Amplitude in degrees
    --chirp-init-freq: Initial frequency in Hz
    --chirp-sweep-rate: Rate of frequency increase in Hz/s
    --chirp-duration: Test duration in seconds"""
    handle_test(ctx, kwargs, 'compare', 'chirp')

# ... existing code ...

for mode_group in [compare, real, sim]:
    @mode_group.command(name='enable')
    @click.argument('servo_ids', nargs=-1, type=int, required=True)
    @click.pass_context
    def enable_servos(ctx, servo_ids):
        """Enable specified servo IDs"""
        config = {'tune': {'mode': ctx.parent.command.name, 'enable_servos': list(servo_ids)}}
        _validate_and_run(config)

    @mode_group.command(name='disable')
    @click.argument('servo_ids', nargs=-1, type=int, required=True)
    @click.pass_context
    def disable_servos(ctx, servo_ids):
        """Disable specified servo IDs"""
        config = {'tune': {'mode': ctx.parent.command.name, 'disable_servos': list(servo_ids)}}
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
    
    # If we're just enabling/disabling servos, don't try to run a test
    if 'enable_servos' in config['tune'] or 'disable_servos' in config['tune']:
        ktune.run_test(None)  # This will just do the setup and servo operations
    else:
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
@click.option('--ip', help='KOS IP address')
@click.option('--actuator-id', type=int, default=11, help='Actuator ID to test')
# Motor parameters
@click.option('--motor-name', default="sts3215", help='Motor model name')
@click.option('--winding-resistance', type=float, default=2.1, help='Motor winding resistance (ohms)')
@click.option('--torque-constant', type=float, default=0.0955, help='Motor torque constant (Nm/A)')
# Control parameters
@click.option('--kp', type=float, default=32.0, help='Position gain')
@click.option('--error-gain', type=float, default=1.0, help='Error gain for system ID')
# Pendulum parameters
@click.option('--mass', type=float, help='Pendulum mass (kg)')  # Removed required=True
@click.option('--length', type=float, help='Pendulum length (m)')  # Removed required=True
# Test configuration
@click.option('--trajectory', type=str, help='Trajectory type: lift_and_drop, sin_time_square, up_and_down, sin_sin, brutal, nothing')
@click.option('--sample-rate', type=float, default=100.0, help='Data collection rate (Hz)')
@click.pass_context
def pendulum(ctx, **kwargs):
    """Run pendulum system identification experiment"""
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

    base_config = cfg.get('sysid', {})
    
    # Update with CLI args, excluding config file path
    cli_args = {k: v for k, v in kwargs.items() if k != 'config' and v is not None}
    base_config.update(cli_args)
    
    # Verify required parameters are present either in config or CLI
    required_params = ['mass', 'length']
    missing_params = [param for param in required_params if param not in base_config]
    if missing_params:
        click.echo(f"Error: Missing required parameters: {', '.join(missing_params)}", err=True)
        raise click.Abort()
    
    # If using config file with multiple tests
    if 'trajectories' in base_config and 'kp_values' in base_config:
        trajectories = base_config.get('trajectories', [])
        kp_values = base_config.get('kp_values', [])
        repetitions = base_config.get('repetitions', 1)

        for trajectory in trajectories:
            for kp in kp_values:
                for rep in range(repetitions):
                    click.echo(f"Running test: trajectory={trajectory}, kp={kp}, repetition={rep+1}/{repetitions}")
                    
                    # Create test config with current parameters
                    test_config = {'sysid': base_config.copy()}
                    test_config['sysid']['trajectory'] = trajectory
                    test_config['sysid']['kp'] = kp
                    
                    # Run the test
                    _validate_and_run_sysid(test_config)
    
    # If using CLI parameters or simple config file
    else:
        # Update config with CLI arguments (CLI args take precedence)
        base_config.update({k: v for k, v in kwargs.items() if v is not None})
        cfg['sysid'] = base_config
        
        # Validate and run single test
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