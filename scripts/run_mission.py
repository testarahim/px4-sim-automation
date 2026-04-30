import asyncio
from mavsdk import System

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Bağlanıyor...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Bağlandı!")
            break

    print("Arm ediliyor...")
    await drone.action.arm()

    print("Takeoff...")
    await drone.action.takeoff()

    await asyncio.sleep(10)

    print("Landing...")
    await drone.action.land()

if __name__ == "__main__":
    asyncio.run(run())
