from __future__ import annotations

import atexit
import os
import subprocess
import sys
from importlib.resources import files

from mavsdk import System
from mavsdk import bin as mavsdk_bin


class OwnedMavsdkSystem(System):
    """One MAVSDK System whose server survives terminal SIGINT for landing."""

    def __init__(self, port: int = 50051, sysid: int = 245, compid: int = 190) -> None:
        super().__init__(mavsdk_server_address="localhost", port=port, sysid=sysid, compid=compid)
        self._owned_process: subprocess.Popen | None = None
        self._owned_port = port
        self._owned_sysid = sysid
        self._owned_compid = compid
        atexit.register(self.close)

    async def connect(self, system_address: str | None = None) -> None:
        if self._owned_process is not None:
            raise RuntimeError("Owned MAVSDK server is already running")
        executable = "mavsdk_server.exe" if sys.platform.startswith("win") else "mavsdk_server"
        backend = files(mavsdk_bin).joinpath(executable)
        arguments = [
            os.fspath(backend),
            "-p",
            str(self._owned_port),
            "--sysid",
            str(self._owned_sysid),
            "--compid",
            str(self._owned_compid),
        ]
        if system_address:
            arguments.append(system_address)
        self._owned_process = subprocess.Popen(
            arguments,
            shell=False,
            start_new_session=True,
        )
        await super().connect()

    def close(self) -> None:
        process = self._owned_process
        if process is None:
            return
        self._owned_process = None
        if process.poll() is not None:
            return
        process.terminate()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3.0)
