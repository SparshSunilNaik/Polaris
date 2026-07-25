import asyncio
from dataclasses import dataclass

from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed


@dataclass(frozen=True)
class Motion:
    name: str
    forward_m_s: float
    right_m_s: float
    down_m_s: float
    yaw_rate_deg_s: float
    duration_s: float


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


async def send_motion(drone: System, motion: Motion) -> None:
    print(f"\n{motion.name}")

    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(
            motion.forward_m_s,
            motion.right_m_s,
            motion.down_m_s,
            motion.yaw_rate_deg_s,
        )
    )

    await asyncio.sleep(motion.duration_s)


async def stop_motion(drone: System, duration_s: float = 2.0) -> None:
    print("Hovering...")

    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(
            0.0,
            0.0,
            0.0,
            0.0,
        )
    )

    await asyncio.sleep(duration_s)


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

        print("Allowing takeoff to stabilize...")
        await asyncio.sleep(6)

        # PX4 requires an initial setpoint before starting Offboard mode.
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(
                0.0,
                0.0,
                0.0,
                0.0,
            )
        )

        print("Starting Offboard mode...")
        await drone.offboard.start()

        motions = [
            Motion(
                name="Moving forward",
                forward_m_s=0.7,
                right_m_s=0.0,
                down_m_s=0.0,
                yaw_rate_deg_s=0.0,
                duration_s=3.0,
            ),
            Motion(
                name="Moving backward",
                forward_m_s=-0.7,
                right_m_s=0.0,
                down_m_s=0.0,
                yaw_rate_deg_s=0.0,
                duration_s=3.0,
            ),
            Motion(
                name="Moving right",
                forward_m_s=0.0,
                right_m_s=0.7,
                down_m_s=0.0,
                yaw_rate_deg_s=0.0,
                duration_s=3.0,
            ),
            Motion(
                name="Moving left",
                forward_m_s=0.0,
                right_m_s=-0.7,
                down_m_s=0.0,
                yaw_rate_deg_s=0.0,
                duration_s=3.0,
            ),
            Motion(
                name="Climbing",
                forward_m_s=0.0,
                right_m_s=0.0,
                down_m_s=-0.4,
                yaw_rate_deg_s=0.0,
                duration_s=2.0,
            ),
            Motion(
                name="Descending",
                forward_m_s=0.0,
                right_m_s=0.0,
                down_m_s=0.4,
                yaw_rate_deg_s=0.0,
                duration_s=2.0,
            ),
            Motion(
                name="Yawing clockwise",
                forward_m_s=0.0,
                right_m_s=0.0,
                down_m_s=0.0,
                yaw_rate_deg_s=25.0,
                duration_s=3.0,
            ),
            Motion(
                name="Yawing counter-clockwise",
                forward_m_s=0.0,
                right_m_s=0.0,
                down_m_s=0.0,
                yaw_rate_deg_s=-25.0,
                duration_s=3.0,
            ),
        ]

        for motion in motions:
            await send_motion(drone, motion)
            await stop_motion(drone, duration_s=1.5)

        print("\nStopping Offboard mode...")
        await stop_motion(drone)
        await drone.offboard.stop()

        print("Landing...")
        await drone.action.land()
        await wait_until_landed(drone)

        print("Movement milestone complete.")

    except (ActionError, OffboardError) as exc:
        print(f"Flight command failed: {exc}")

        # Best-effort safety response.
        try:
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
            )
        except Exception:
            pass

        try:
            await drone.action.land()
        except Exception:
            pass

        raise


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nScript interrupted.")
