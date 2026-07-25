import asyncio

from mavsdk import System
from mavsdk.action import ActionError


async def wait_for_connection(drone: System) -> None:
    print("Waiting for PX4 connection...")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected to PX4.")
            return


async def wait_until_armable(drone: System) -> None:
    print("Waiting for vehicle health...")

    async for health in drone.telemetry.health():
        print(
            f"local_position={health.is_local_position_ok}, "
            f"home_position={health.is_home_position_ok}, "
            f"armable={health.is_armable}"
        )

        if health.is_local_position_ok and health.is_armable:
            print("Vehicle is ready.")
            return

        await asyncio.sleep(0.5)


async def wait_until_landed(drone: System) -> None:
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            return


async def main() -> None:
    drone = System()
    await drone.connect(system_address="udp://:14540")

    await wait_for_connection(drone)
    await wait_until_armable(drone)

    try:
        print("Setting takeoff altitude to 2.5 m...")
        await drone.action.set_takeoff_altitude(2.5)

        print("Arming...")
        await drone.action.arm()

        print("Taking off...")
        await drone.action.takeoff()

        print("Hovering for 8 seconds...")
        await asyncio.sleep(8)

        print("Landing...")
        await drone.action.land()

        await wait_until_landed(drone)

        print("Landed successfully.")

    except ActionError as exc:
        print(f"PX4 action failed: {exc}")
        raise


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nFlight script interrupted.")
