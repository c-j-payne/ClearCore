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

        
            

    async def close(self):
        LOGGER.info("close") 
        await self.serial_port.do_command({"message": f"c{self.motor_id}"})
        time.sleep(0.1)
        await self.serial_port.get_readings()
        time.sleep(0.1)
        await self.reset_zero_position(0)
        

    async def do_command(self, command: Mapping[str, ValueTypes], *, timeout: Optional[float] = None, **kwargs) -> Mapping[str, ValueTypes]:        
        return ""

    #clearcore needs steps per min
    def steps_per_minute(self, rpm: float) -> int:
        return int(rpm * (1 / 60) * (self.steps_per_revolution / 1))
        


    #enable check

    async def enable(self):
        if self.ispowered==False:
            await self.serial_port.do_command({"message": f"e{self.motor_id}"})
            time.sleep(0.2)
            await self.serial_port.get_readings()
            self.ispowered=True
            time.sleep(0.1)

  
    async def set_power(
            self,
            power: float,
            *,
            extra: Optional[Dict[str, Any]] = None,
            timeout: Optional[float] = None,
            **kwargs,
    ):
        
        if self.ispowered==False:
            await self.enable()      
        else:
            pass

        power = int(power*self.max_rpm)        
        await self.serial_port.do_command({"message": f"v{self.motor_id} {power}"})
        time.sleep(0.1)
        await self.serial_port.get_readings()

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
        if self.ispowered==False:   
            await self.enable()         
        else:
            pass
        
        # Construct the command to set velocity
        await self.serial_port.do_command({"message": f"m{self.motor_id} {position_revolutions}"})
        time.sleep(0.2)
        await self.serial_port.get_readings()

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
        time.sleep(0.1)
        await self.serial_port.get_readings() 

        return

    async def get_position(
            self,
            *,
            extra: Optional[Dict[str, Any]] = None,
            timeout: Optional[float] = None,
            **kwargs,
    ) -> float:

        with self.serial_lock:
            await self.serial_port.do_command({"message": f"q{self.motor_id}p"})
            time.sleep(0.1)
            position_message = await self.serial_port.get_readings()
            if not position_message:
                while True:
                    position_message = await self.serial_port.get_readings()
                    if position_message:
                        break
            position = position_message["reading"][1]
            pattern = r"\(steps\) (\d+)"
            match = re.search(pattern, position)
            steps = int(match.group(1))
        return steps

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

        #disable
        #await self.serial_port.do_command({"message": f"e{self.motor_id}"})
        with self.serial_lock:
            await self.serial_port.do_command({"message": f"v{self.motor_id} 0"})
            time.sleep(0.2)
            await self.serial_port.get_readings()
            time.sleep(0.1)        
            await self.serial_port.do_command({"message": f"d{self.motor_id}"})
            await self.serial_port.get_readings()
            self.ispowered=False
            return None

    async def is_powered(
            self,
            *,
            extra: Optional[Dict[str, Any]] = None,
            timeout: Optional[float] = None,
            **kwargs,
    ) -> Tuple[bool, float]:

        #await self.serial_port.do_command({"message": f"q{self.motor_id}v"})
        #x = await self.serial_port.get_readings()
        #LOGGER.info(x)


        return (self.ispowered,1.0)

    async def is_moving(self) -> bool:
        #helpcommand = {"message": "h"}
        #LOGGER.info(helpcommand)
        #await self.serial_port.do_command(helpcommand)
        return False

    async def go_to(
            self,
            rpm: float,
            position_revolutions: float,
            *,
            extra: Optional[Dict[str, Any]] = None,
            timeout: Optional[float] = None,
            **kwargs,
    ):
        current_position = await self.get_position()
        delta_position = position_revolutions - current_position
        
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
            with self.serial_lock:            
                await self.serial_port.do_command({"message": f"l{self.motor_id}v {rpm}"})
                time.sleep(0.2)
                await self.serial_port.get_readings()


            # Construct the command to set position
            with self.serial_lock:
                await self.serial_port.do_command({"message": f"m{self.motor_id} {delta_position}"})
                time.sleep(0.1)
                await self.serial_port.get_readings()
                time.sleep(0.1)
                current_position = await self.get_position()
                time.sleep(0.1)
                if current_position != position_revolutions:
                    while True:
                        current_position = await self.get_position()
                        time.sleep(0.1)
                        if current_position == position_revolutions:
                            time.sleep(0.1)
                            
                            break
                    await self.stop()               
        return 