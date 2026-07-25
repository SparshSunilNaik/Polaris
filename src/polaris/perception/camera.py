from __future__ import annotations

from collections import deque
from threading import Lock
from time import monotonic

from .dependencies import add_homebrew_python_site
from .models import CameraFrame

add_homebrew_python_site()

import cv2  # noqa: E402
import numpy as np  # noqa: E402
from gz.msgs10.image_pb2 import Image  # noqa: E402
from gz.transport13 import Node  # noqa: E402


DEFAULT_CAMERA_TOPIC = (
    "/world/default/model/x500_mono_cam_0/link/camera_link/"
    "sensor/camera/image"
)


def decode_image_message(message: Image, sequence: int = 0) -> CameraFrame:
    format_field = Image.DESCRIPTOR.fields_by_name["pixel_format_type"]
    format_value = format_field.enum_type.values_by_number.get(
        message.pixel_format_type
    )
    if format_value is None:
        raise ValueError(f"Unknown Gazebo pixel format {message.pixel_format_type}")
    pixel_format = format_value.name

    conversions = {
        "RGB_INT8": (3, cv2.COLOR_RGB2BGR),
        "BGR_INT8": (3, None),
        "RGBA_INT8": (4, cv2.COLOR_RGBA2BGR),
        "BGRA_INT8": (4, cv2.COLOR_BGRA2BGR),
        "L_INT8": (1, cv2.COLOR_GRAY2BGR),
    }
    if pixel_format not in conversions:
        raise ValueError(f"Unsupported Gazebo pixel format {pixel_format}")

    channels, conversion = conversions[pixel_format]
    packed_width = message.width * channels
    if message.step < packed_width:
        raise ValueError(
            f"Invalid image step {message.step} for {message.width}x{channels}"
        )
    expected_size = message.step * message.height
    if len(message.data) < expected_size:
        raise ValueError(
            f"Image data is truncated: {len(message.data)} < {expected_size}"
        )

    rows = np.frombuffer(message.data, dtype=np.uint8, count=expected_size).reshape(
        message.height, message.step
    )
    packed = rows[:, :packed_width]
    if channels == 1:
        image = packed.reshape(message.height, message.width)
    else:
        image = packed.reshape(message.height, message.width, channels)
    image_bgr = cv2.cvtColor(image, conversion) if conversion is not None else image.copy()

    stamp = message.header.stamp
    source_timestamp = stamp.sec + stamp.nsec / 1_000_000_000
    return CameraFrame(
        image_bgr=image_bgr,
        timestamp_monotonic=monotonic(),
        source_timestamp=source_timestamp,
        sequence=sequence,
        width=message.width,
        height=message.height,
        pixel_format=pixel_format,
    )


class GazeboCameraSubscriber:
    def __init__(self, topic: str | None = None) -> None:
        self.topic = topic or DEFAULT_CAMERA_TOPIC
        self._requested_topic = topic
        self._subscribed_topic: str | None = None
        self._node = Node()
        self._lock = Lock()
        self._latest: CameraFrame | None = None
        self._sequence = 0
        self._arrival_times: deque[float] = deque(maxlen=120)
        self._last_error: str | None = None
        self._started = False
        self._last_discovery_attempt = 0.0

    def start(self) -> None:
        if self._started:
            return
        self._started = True
        self._ensure_subscription()

    def close(self) -> None:
        if self._subscribed_topic is not None:
            self._node.unsubscribe(self._subscribed_topic)
            self._subscribed_topic = None
        self._started = False

    def latest_frame(self) -> CameraFrame | None:
        self._ensure_subscription()
        with self._lock:
            return self._latest

    @property
    def fps(self) -> float:
        with self._lock:
            if len(self._arrival_times) < 2:
                return 0.0
            elapsed = self._arrival_times[-1] - self._arrival_times[0]
            return (len(self._arrival_times) - 1) / elapsed if elapsed > 0 else 0.0

    @property
    def last_error(self) -> str | None:
        with self._lock:
            return self._last_error

    def _on_image(self, message: Image) -> None:
        try:
            with self._lock:
                sequence = self._sequence + 1
            frame = decode_image_message(message, sequence)
        except (ValueError, cv2.error) as exc:
            with self._lock:
                self._last_error = str(exc)
            return

        with self._lock:
            self._sequence = sequence
            self._latest = frame
            self._arrival_times.append(frame.timestamp_monotonic)
            self._last_error = None

    def _ensure_subscription(self) -> None:
        if not self._started or self._subscribed_topic is not None:
            return
        now = monotonic()
        if now - self._last_discovery_attempt < 1.0:
            return
        self._last_discovery_attempt = now

        topic = self._requested_topic
        if topic is None:
            suffix = (
                "/model/x500_mono_cam_0/link/camera_link/"
                "sensor/camera/image"
            )
            matches = [name for name in self._node.topic_list() if name.endswith(suffix)]
            if len(matches) == 1:
                topic = matches[0]
            elif len(matches) > 1:
                with self._lock:
                    self._last_error = (
                        "Multiple X500 camera topics found; select one with --topic"
                    )
                return
            else:
                with self._lock:
                    self._last_error = "Waiting for X500 camera topic"
                return

        self.topic = topic
        self._node.subscribe(Image, topic, self._on_image)
        self._subscribed_topic = topic
