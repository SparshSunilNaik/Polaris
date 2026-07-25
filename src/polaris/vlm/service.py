from __future__ import annotations

import base64
import hashlib
import json
import socket
import time
import urllib.error
import urllib.request
from collections import OrderedDict
from threading import RLock
from typing import Any, Mapping, Protocol

from .models import VLMConfig, VLMHealth, VLMResult, VLMStatus


SYSTEM_PROMPT = """You are the deterministic scene-understanding component for Project Polaris.
Analyze only the supplied image. Do not converse, explain, use Markdown, or add keys.
Return ONLY one valid JSON object with exactly these keys:
terrain_type (string), visible_landmarks (array of strings), target_visible (boolean),
scene_confidence (number from 0 to 1), texture_quality (number from 0 to 1),
scene_summary (concise string). Use "UNKNOWN" for an unknown terrain type and
conservative confidence values when uncertain. Never return an empty string."""

USER_PROMPT = "Analyze this scene and return the required JSON object only."

RESPONSE_SCHEMA = {
    "type": "object",
    "properties": {
        "terrain_type": {"type": "string", "minLength": 1, "maxLength": 100},
        "visible_landmarks": {
            "type": "array",
            "items": {"type": "string", "minLength": 1, "maxLength": 100},
            "maxItems": 32,
        },
        "target_visible": {"type": "boolean"},
        "scene_confidence": {"type": "number", "minimum": 0, "maximum": 1},
        "texture_quality": {"type": "number", "minimum": 0, "maximum": 1},
        "scene_summary": {"type": "string", "minLength": 1, "maxLength": 1000},
    },
    "required": [
        "terrain_type",
        "visible_landmarks",
        "target_visible",
        "scene_confidence",
        "texture_quality",
        "scene_summary",
    ],
    "additionalProperties": False,
}

_RESPONSE_FIELDS = frozenset(RESPONSE_SCHEMA["required"])


class _Transport(Protocol):
    def request_json(
        self,
        path: str,
        payload: Mapping[str, Any] | None,
        timeout_s: float,
    ) -> Mapping[str, Any]: ...


class _InvalidResponse(Exception):
    pass


class _HTTPFailure(Exception):
    def __init__(self, status_code: int) -> None:
        super().__init__(f"Ollama HTTP status {status_code}")
        self.status_code = status_code

    @property
    def retryable(self) -> bool:
        return self.status_code == 429 or self.status_code >= 500


class _OllamaHTTPTransport:
    def __init__(self, endpoint: str) -> None:
        self._endpoint = endpoint.rstrip("/")

    def request_json(
        self,
        path: str,
        payload: Mapping[str, Any] | None,
        timeout_s: float,
    ) -> Mapping[str, Any]:
        data = None if payload is None else json.dumps(payload).encode("utf-8")
        request = urllib.request.Request(
            self._endpoint + path,
            data=data,
            method="GET" if payload is None else "POST",
            headers={"Accept": "application/json", "Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(request, timeout=timeout_s) as response:
                body = response.read()
        except urllib.error.HTTPError as exc:
            raise _HTTPFailure(exc.code) from exc
        except (TimeoutError, socket.timeout) as exc:
            raise TimeoutError from exc
        except urllib.error.URLError as exc:
            if isinstance(getattr(exc, "reason", None), (TimeoutError, socket.timeout)):
                raise TimeoutError from exc
            raise OSError from exc
        except OSError as exc:
            raise OSError from exc
        try:
            result = json.loads(body)
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise _InvalidResponse from exc
        if not isinstance(result, dict):
            raise _InvalidResponse
        return result


class VLMService:
    """Thread-safe, read-only scene understanding through local Ollama."""

    def __init__(
        self,
        config: VLMConfig | None = None,
        *,
        _transport: _Transport | None = None,
    ) -> None:
        self.config = config or VLMConfig()
        self.config.validate()
        self._transport = _transport or _OllamaHTTPTransport(self.config.endpoint)
        self._lock = RLock()
        self._cache: OrderedDict[str, tuple[float, VLMResult]] = OrderedDict()

    def is_available(self) -> bool:
        return self.health_check().available

    def verify_model(self) -> bool:
        return self.health_check().model_available

    def health_check(self) -> VLMHealth:
        with self._lock:
            try:
                payload = self._request_with_retry(
                    "/api/tags",
                    None,
                    self.config.health_timeout_s,
                )
                models = self._parse_models(payload)
            except (OSError, TimeoutError, _HTTPFailure, _InvalidResponse):
                return VLMHealth(False, False, self.config.model)
            return VLMHealth(
                reachable=True,
                model_available=self.config.model in models,
                model=self.config.model,
                installed_models=models,
            )

    def analyze(self, image: bytes | bytearray | memoryview) -> VLMResult:
        image_bytes = self._image_bytes(image)
        cache_key = hashlib.sha256(image_bytes).hexdigest()
        with self._lock:
            cached = self._cached(cache_key)
            if cached is not None:
                return cached
            options = {"temperature": 0, "seed": 0}
            if self.config.ollama_num_gpu is not None:
                options["num_gpu"] = self.config.ollama_num_gpu
            request = {
                "model": self.config.model,
                "stream": False,
                "format": RESPONSE_SCHEMA,
                "options": options,
                "messages": [
                    {"role": "system", "content": SYSTEM_PROMPT},
                    {
                        "role": "user",
                        "content": USER_PROMPT,
                        "images": [base64.b64encode(image_bytes).decode("ascii")],
                    },
                ],
            }
            try:
                payload = self._request_with_retry(
                    "/api/chat",
                    request,
                    self.config.inference_timeout_s,
                )
                result = self._parse_result(payload)
            except TimeoutError:
                return VLMResult.degraded(VLMStatus.TIMEOUT, self.config.model)
            except _InvalidResponse:
                return VLMResult.degraded(VLMStatus.INVALID_RESPONSE, self.config.model)
            except _HTTPFailure as exc:
                status = (
                    VLMStatus.MODEL_UNAVAILABLE
                    if exc.status_code == 404
                    else VLMStatus.UNAVAILABLE
                )
                return VLMResult.degraded(status, self.config.model)
            except OSError:
                return VLMResult.degraded(VLMStatus.UNAVAILABLE, self.config.model)
            self._store(cache_key, result)
            return result

    def _request_with_retry(
        self,
        path: str,
        payload: Mapping[str, Any] | None,
        timeout_s: float,
    ) -> Mapping[str, Any]:
        for attempt in range(self.config.maximum_retries + 1):
            try:
                return self._transport.request_json(path, payload, timeout_s)
            except _HTTPFailure as exc:
                if not exc.retryable or attempt >= self.config.maximum_retries:
                    raise
            except (OSError, TimeoutError):
                if attempt >= self.config.maximum_retries:
                    raise
            if self.config.retry_delay_s:
                time.sleep(self.config.retry_delay_s)
        raise AssertionError("Retry loop exited without returning or raising")

    def _parse_models(self, payload: Mapping[str, Any]) -> tuple[str, ...]:
        values = payload.get("models")
        if not isinstance(values, list):
            raise _InvalidResponse
        names: list[str] = []
        for item in values:
            if not isinstance(item, dict):
                raise _InvalidResponse
            name = item.get("name", item.get("model"))
            if not isinstance(name, str) or not name:
                raise _InvalidResponse
            names.append(name)
        return tuple(names)

    def _parse_result(self, payload: Mapping[str, Any]) -> VLMResult:
        message = payload.get("message")
        if not isinstance(message, dict) or not isinstance(message.get("content"), str):
            raise _InvalidResponse
        try:
            values = json.loads(message["content"])
        except json.JSONDecodeError as exc:
            raise _InvalidResponse from exc
        if not isinstance(values, dict) or set(values) != _RESPONSE_FIELDS:
            raise _InvalidResponse
        try:
            return VLMResult(
                status=VLMStatus.SUCCESS,
                terrain_type=self._string(values["terrain_type"]),
                visible_landmarks=self._strings(values["visible_landmarks"]),
                target_visible=self._boolean(values["target_visible"]),
                scene_confidence=self._number(values["scene_confidence"]),
                texture_quality=self._number(values["texture_quality"]),
                scene_summary=self._string(values["scene_summary"]),
                model=self.config.model,
            )
        except (TypeError, ValueError) as exc:
            raise _InvalidResponse from exc

    @staticmethod
    def _image_bytes(image: bytes | bytearray | memoryview) -> bytes:
        if not isinstance(image, (bytes, bytearray, memoryview)):
            raise TypeError("image must be encoded bytes")
        result = bytes(image)
        if not result:
            raise ValueError("image must not be empty")
        return result

    @staticmethod
    def _string(value: Any) -> str:
        if not isinstance(value, str):
            raise TypeError
        return value

    @staticmethod
    def _strings(value: Any) -> tuple[str, ...]:
        if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
            raise TypeError
        return tuple(value)

    @staticmethod
    def _boolean(value: Any) -> bool:
        if not isinstance(value, bool):
            raise TypeError
        return value

    @staticmethod
    def _number(value: Any) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise TypeError
        return float(value)

    def _cached(self, key: str) -> VLMResult | None:
        item = self._cache.get(key)
        if item is None:
            return None
        stored_at, result = item
        if time.monotonic() - stored_at > self.config.cache_ttl_s:
            del self._cache[key]
            return None
        self._cache.move_to_end(key)
        return result

    def _store(self, key: str, result: VLMResult) -> None:
        if self.config.cache_capacity == 0 or self.config.cache_ttl_s == 0:
            return
        self._cache[key] = (time.monotonic(), result)
        self._cache.move_to_end(key)
        while len(self._cache) > self.config.cache_capacity:
            self._cache.popitem(last=False)
