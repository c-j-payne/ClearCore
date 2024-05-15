from viam.components.motor import Motor
from viam.resource.registry import Registry, ResourceCreatorRegistration
from .clearcore import clearcore

#from previous module: 
Registry.register_resource_creator(Motor.SUBTYPE, clearcore.MODEL, ResourceCreatorRegistration(clearcore.new, clearcore.validate_config))