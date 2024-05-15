import asyncio

from viam.module.module import Module
from viam.components.motor import Motor
from viam.resource.registry import Registry, ResourceCreatorRegistration

from clearcore import ClearCore 


async def main():
    """This function creates and starts a new module, after adding all desired resources.
    Resources must be pre-registered. For an example, see the `__init__.py` file.
    """
    Registry.register_resource_creator(Motor.SUBTYPE, ClearCore.MODEL, ResourceCreatorRegistration(ClearCore.new))
    module = Module.from_args()
    module.add_model_from_registry(Motor.SUBTYPE, ClearCore.MODEL)
    await module.start()


if __name__ == "__main__":
    asyncio.run(main())