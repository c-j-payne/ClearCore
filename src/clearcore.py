import serial
import time
import threading
import asyncio
import re
from typing import Any, ClassVar, Dict, Mapping, Optional, Sequence, List, cast, Tuple
from viam.components.motor import Motor
from viam.components.sensor import Sensor
from viam.logging import getLogger
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes, struct_to_dict
LOGGER = getLogger(__name__)

class ClearCore(Motor):
    MODEL: ClassVar[Model] = Model(ModelFamily("c-j-payne", "clear-core"), "controller")
    serial_baud_rate: int
    serial_path: str 
    serial_timeout: int
    serial_lock: threading.Lock()
    ser: Optional[serial.Serial] = None    

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        return []

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> "ClearCore":
        motor = cls(config.name)
        motor.reconfigure(config, dependencies)
        return motor

    def __init__(self, name: str):
        super().__init__(name)
        self.serial_lock = threading.Lock()
        self.ispowered = False
        self.steps = 0
        self.current_rpm = 0

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):

        # Error handling for serial port
        try:
            serial_dependency = Sensor.get_resource_name(config.attributes.fields['serial'].string_value)
            self.serial_port = cast(Sensor, dependencies[serial_dependency])
            LOGGER.info("Serial port dependency logged")
        except (KeyError, TypeError, AttributeError) as e:
            raise ValueError("Error handling serial port dependency:", e)

        # Error handling for steps_per_revolution
        try:
            steps_per_revolution = config.attributes.fields.get("steps_per_revolution", {}).number_value
            if steps_per_revolution is None:
                raise ValueError("'steps_per_revolution' field is missing in the config")
            self.steps_per_revolution = int(steps_per_revolution)
            LOGGER.info(f"steps_per_revolution: {self.steps_per_revolution}")
        except (ValueError, TypeError) as e:
            raise ValueError("'steps_per_revolution' must be an integer:", e)

        # Error handling for motor_id
        try:
            motor_id = config.attributes.fields.get("motor_id", {}).number_value
            if motor_id is None:
                raise ValueError("'motor_id' field is missing in the config")
            self.motor_id = int(motor_id)
            LOGGER.info(f"motor_id: {self.motor_id}")
        except (ValueError, TypeError) as e:
            raise ValueError("'motor_id' must be an integer:", e)

        # Error handling for max_rpm
        try:
            max_rpm = config.attributes.fields.get("max_rpm", {}).number_value
            if max_rpm is None:
                raise ValueError("'max_rpm' field is missing in the config")
            self.max_rpm = int(max_rpm)
            LOGGER.info(f"max_rpm: {self.max_rpm}")
        except (ValueError, TypeError) as e:
            raise ValueError("'max_rpm' must be an integer:", e)

        '''try:
            zero_position_on_close = config.attributes.fields.get("zero_position_on_close", {}).number_value
            if max_rpm is None:
                pass
            self.zero_position_on_close = int(zero_position_on_close)
        except (ValueError, TypeError) as e:
            raise ValueError("'max_rpm' must be an integer:", e)'''
                  

    async def close(self):
        LOGGER.info("close")
        #clear all errors
        await self.serial_port.do_command({"message": f"c{self.motor_id}"})
        await asyncio.sleep(0.2)
        #zero the position
        await self.serial_port.do_command({"message": f"z{self.motor_id}"})
        await asyncio.sleep(0.2)


    #clearcore needs steps per min
    def steps_per_minute(self, rpm: float) -> int:
        self.current_rpm = rpm
        return int(rpm * (1 / 60) * (self.steps_per_revolution / 1))

    async def do_command(self, command: Mapping[str, ValueTypes], *, timeout: Optional[float] = None, **kwargs) -> Mapping[str, ValueTypes]:        
        return ""

    #enable check
    async def enable(self):
        if self.ispowered==False:
            #with self.serial_lock:
            await self.serial_port.do_command({"message": f"e{self.motor_id}"})
            self.ispowered=True
            await asyncio.sleep(0.3)

  
    async def set_power(
            self,
            power: float,
            *,
            extra: Optional[Dict[str, Any]] = None,
            timeout: Optional[float] = None,
            **kwargs,
    ):        
        #convert power to rpm fraction
        power = int(power*self.max_rpm)       
        self.current_rpm = power         
        
        #enable if not already
        if self.ispowered==False:
            await self.enable()      
        else:
            pass

        #send velocity command
        await self.serial_port.do_command({"message": f"v{self.motor_id} {power}"})
        
        return   
    
    async def go_for(
        self,
        rpm: float,
        position_revolutions: float,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ):

        # Calculate the target position relative to the current position
        delta_position = int(position_revolutions * self.steps_per_revolution)
        initial_position = int(self.steps)

        #check that a speed is selected
        if rpm <= 0:
            pass
        
        else:
            #check that there is a positional error to correct       
            if delta_position != 0:

                # Enable motor if not powered
                if not self.ispowered:
                    await self.enable()

                # Convert RPM to steps per minute
                rpm = self.steps_per_minute(rpm)
                rpm = int(rpm)
                self.current_rpm = rpm

                #set speed limit
                #send velocity command, check max
                if rpm < self.max_rpm:  
                    await self.serial_port.do_command({"message": f"l{self.motor_id}v {rpm}"})
                    if rpm <= 0:
                        pass
                else:
                    await self.serial_port.do_command({"message": f"l{self.motor_id}v {self.max_rpm}"})
                await asyncio.sleep(0.1)

                # Move by the relative position
                await self.serial_port.do_command({"message": f"m{self.motor_id} {delta_position}"})
                await asyncio.sleep(0.1)

                # Wait until the movement is completed
                while True:
                    await self.get_position()
                    current_position = int(self.steps)
                    relative_position = current_position - initial_position
                    if relative_position == delta_position:
                        break
                    await asyncio.sleep(0.1)

                # Stop the motor
                await self.stop()

        return


    async def reset_zero_position(
            self,
            offset: float,
            *,
            extra: Optional[Dict[str, Any]] = None,
            timeout: Optional[float] = None,
            **kwargs,
    ):
        await self.serial_port.do_command({"message": f"z{self.motor_id}"})
        await asyncio.sleep(0.1)
        return


    async def get_position(
            self,
            *,
            extra: Optional[Dict[str, Any]] = None,
            timeout: Optional[float] = None,
            **kwargs,
    ) -> float:

        await self.serial_port.do_command({"message": f"q{self.motor_id}p"})
        await asyncio.sleep(0.05)
        reading = await self.serial_port.get_readings()
        messages = reading["reading"]
        await asyncio.sleep(0.05)
        
        # Is there anything in the serial buffer?
        if len(messages) > 0:

            # Loop through messages to extract data
            for message in messages:

                # Is it position feedback?
                if 'is in position' in message:                    
                    position = reading["reading"][1]
                    pattern = r"\(steps\) (-?\d+)"
                    if position is not None:
                        match = re.search(pattern, position)
                        if match:
                            self.steps = int(match.group(1))

        position_revolutions = self.steps / self.steps_per_revolution

        return position_revolutions


    async def get_properties(
            self,
            *,
            extra: Optional[Dict[str, Any]] = None,
            timeout: Optional[float] = None,
            **kwargs,
    ) -> Motor.Properties:
        return Motor.Properties(True)


    async def stop(
            self,
            *,
            extra: Optional[Dict[str, Any]] = None,
            timeout: Optional[float] = None,
            **kwargs,
    ):        
        self.ispowered=False
        
        #zero velocity first
        
        await self.serial_port.do_command({"message": f"v{self.motor_id} 0"})
        await asyncio.sleep(0.3)
        reading = await self.serial_port.get_readings()
        LOGGER.info(reading)
        
        #then disable
        await self.serial_port.do_command({"message": f"d{self.motor_id}"})
        await asyncio.sleep(0.3)
        reading = await self.serial_port.get_readings()
        LOGGER.info(reading)

        return None


    async def is_powered(
            self,
            *,
            extra: Optional[Dict[str, Any]] = None,
            timeout: Optional[float] = None,
            **kwargs,
    ) -> Tuple[bool, float]:
        return (self.ispowered,1.0)


    async def is_moving(self) -> bool:
        return self.ispowered


    async def go_to(
            self,
            rpm: float,
            position_revolutions: float,
            *,
            extra: Optional[Dict[str, Any]] = None,
            timeout: Optional[float] = None,
            **kwargs,
    ):       

        await self.get_position()
        current_position = int(self.steps)
        position_revolutions = int(position_revolutions*self.steps_per_revolution)
        delta_position = position_revolutions - current_position
        delta_position = int(delta_position)

        #check that a speed is selected
        if rpm <= 0:
            pass
        
        else:
            #check that there is a positional error to correct       
            if delta_position != 0:

                #enable        
                if self.ispowered==False:   
                    await self.enable()         
                else:
                    pass
                
                #convert to rpm
                rpm = self.steps_per_minute(rpm)
                rpm = int(rpm)                

                #set speed limit
                #send velocity command, check max
                if rpm < self.max_rpm:  
                    await self.serial_port.do_command({"message": f"l{self.motor_id}v {rpm}"})                
                else:
                    await self.serial_port.do_command({"message": f"l{self.motor_id}v {self.max_rpm}"})
                await asyncio.sleep(0.1)

                # Construct the command to set position
                await self.serial_port.do_command({"message": f"m{self.motor_id} {delta_position}"})
                await asyncio.sleep(0.1)

                while delta_position != 0:
                    current_position = int(self.steps)
                    delta_position = position_revolutions - current_position
                    delta_position = int(delta_position)
                    await self.get_position()
                    await asyncio.sleep(0.1)                 
                    if delta_position == 0:
                        await asyncio.sleep(0.1)                        
                        break
                await self.stop()   

            else:
                pass

        return 
