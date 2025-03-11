# ktune/config/validation.py
import json
import jsonschema
from pathlib import Path
from typing import Dict, Any, Optional
import logging

logger = logging.getLogger(__name__)

class ConfigValidator:
    """Validates KTune configuration files against JSON schemas"""
    
    def __init__(self):
        self.schemas = {}
        self._load_schemas()

    def _load_schemas(self) -> None:
        """Load all JSON schemas from the schemas directory"""
        schema_dir = Path(__file__).parent / "schemas"
        for schema_file in schema_dir.glob("*.json"):
            with open(schema_file) as f:
                self.schemas[schema_file.stem] = json.load(f)

    def validate(self, config: Dict[str, Any], schema_name: str) -> None:
        """Validate a configuration section against its schema"""
        if schema_name not in self.schemas:
            raise ValueError(f"Unknown schema: {schema_name}")
        
        try:
            jsonschema.validate(config, self.schemas[schema_name])
        except jsonschema.exceptions.ValidationError as e:
            raise ValueError(f"Configuration validation failed for {schema_name}: {str(e)}")

    def validate_all(self, config: Dict[str, Any]) -> None:
        """Validate all sections of a configuration"""
        # Validate tune configuration if present
        if 'tune' in config:
            self.validate(config['tune'], 'tune')
            
        # Validate motor configuration if present
        if 'motor' in config:
            self.validate(config['motor'], 'motor')
            
        # Validate testbed configuration if present
        if 'testbed' in config:
            self.validate(config['testbed'], 'testbed')
            
        # Perform cross-validation between sections
        self._validate_cross_references(config)

    def _validate_cross_references(self, config: Dict[str, Any]) -> None:
        """Validate relationships between different configuration sections"""
        if 'motor' in config and 'testbed' in config:
            # Ensure motor limits are compatible with testbed requirements
            self._validate_motor_testbed_compatibility(config['motor'], config['testbed'])

        if 'tune' in config and 'motor' in config:
            # Ensure tuning parameters are within motor limits
            self._validate_tune_motor_compatibility(config['tune'], config['motor'])

    def _validate_motor_testbed_compatibility(self, motor: Dict, testbed: Dict) -> None:
        """Validate that motor configuration is compatible with testbed requirements"""
        if testbed['type'] == 'pendulum':
            # Check if motor position limits can accommodate pendulum motion
            if 'limits' in motor and 'position' in motor['limits']:
                motor_range = abs(motor['limits']['position']['max'] - 
                                motor['limits']['position']['min'])
                
                # Check each trajectory's amplitude against motor limits
                for traj in testbed.get('trajectories', []):
                    amp = traj.get('parameters', {}).get('amplitude', 0)
                    if amp * 2 > motor_range:
                        raise ValueError(
                            f"Trajectory '{traj['name']}' amplitude ({amp}°) exceeds "
                            f"motor range ({motor_range/2}°)"
                        )

    def _validate_tune_motor_compatibility(self, tune: Dict, motor: Dict) -> None:
        """Validate that tuning parameters are compatible with motor limits"""
        # Check if tuning gains are within motor capabilities
        motor_gains = motor.get('gains', {})
        tune_gains = tune.get('actuator', {}).get('gains', {})
        
        for gain_type in ['kp', 'kd', 'ki']:
            if gain_type in tune_gains and gain_type in motor_gains:
                if tune_gains[gain_type] > motor_gains[gain_type] * 1.5:  # 50% margin
                    logger.warning(
                        f"Tuning {gain_type} ({tune_gains[gain_type]}) is significantly "
                        f"higher than motor default ({motor_gains[gain_type]})"
                    )

    def apply_defaults(self, config: Dict[str, Any]) -> Dict[str, Any]:
        """Apply default values from schemas to configuration"""
        result = config.copy()
        
        for section in ['tune', 'motor', 'testbed']:
            if section in result:
                result[section] = self._apply_schema_defaults(
                    result[section],
                    self.schemas[section]
                )
        
        return result

    def _apply_schema_defaults(self, config: Dict, schema: Dict) -> Dict:
        """Recursively apply defaults from schema to configuration"""
        result = config.copy()
        
        if 'properties' in schema:
            for prop, details in schema['properties'].items():
                if prop not in result and 'default' in details:
                    result[prop] = details['default']
                elif (prop in result and details.get('type') == 'object' 
                      and 'properties' in details):
                    result[prop] = self._apply_schema_defaults(
                        result[prop],
                        details
                    )
                    
        return result