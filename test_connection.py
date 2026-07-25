import asyncio
from mavsdk import System


async def print_health(drone: System) -> None:
    async for health in drone.telemetry.health():
        print(
            "Health:",
            {
                "local_position": health.is_local_position_ok,
                "global_position": health.is_global_position_ok,
                "home_position": health.is_home_position_ok,
                "armable": health.is_armable,
            },
        )
        await asyncio.sleep(1)


async def main() -> None:
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for PX4 connection...")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected to PX4. Keeping heartbeat alive.")
            break

    await print_health(drone)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nDisconnected.")
