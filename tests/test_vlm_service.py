from __future__ import annotations

import json
import sys
import time
import unittest
from dataclasses import FrozenInstanceError, replace
from pathlib import Path
from threading import Lock, Thread

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from polaris.vlm import (  # noqa: E402
    SYSTEM_PROMPT,
    VLMConfig,
    VLMService,
    VLMStatus,
)


SCENE = {
    "terrain_type": "snowfield",
    "visible_landmarks": ["rock", "research station"],
    "target_visible": True,
    "scene_confidence": 0.91,
    "texture_quality": 0.62,
    "scene_summary": "Open snowfield with a rock and station ahead.",
}


def chat_response(scene=None):
    return {"message": {"content": json.dumps(SCENE if scene is None else scene)}}


class FakeTransport:
    def __init__(self, *responses) -> None:
        self.responses = list(responses)
        self.calls = []

    def request_json(self, path, payload, timeout_s):
        self.calls.append((path, payload, timeout_s))
        if not self.responses:
            raise AssertionError("Unexpected transport request")
        response = self.responses.pop(0)
        if isinstance(response, BaseException):
            raise response
        return response


class ConcurrentTransport:
    def __init__(self) -> None:
        self.active = 0
        self.maximum_active = 0
        self.lock = Lock()

    def request_json(self, path, payload, timeout_s):
        with self.lock:
            self.active += 1
            self.maximum_active = max(self.maximum_active, self.active)
        time.sleep(0.02)
        with self.lock:
            self.active -= 1
        return chat_response()


class VLMServiceTests(unittest.TestCase):
    def config(self, **changes) -> VLMConfig:
        return replace(
            VLMConfig(),
            maximum_retries=1,
            retry_delay_s=0.0,
            cache_capacity=0,
            **changes,
        )

    def test_ollama_reachable_and_model_available(self) -> None:
        transport = FakeTransport({"models": [{"name": "qwen2.5vl:7b"}]})
        service = VLMService(self.config(), _transport=transport)
        health = service.health_check()
        self.assertTrue(health.reachable)
        self.assertTrue(health.model_available)
        self.assertTrue(health.available)

    def test_ollama_unavailable_degrades_without_raising(self) -> None:
        transport = FakeTransport(OSError(), OSError())
        service = VLMService(self.config(), _transport=transport)
        result = service.analyze(b"encoded-image")
        self.assertEqual(result.status, VLMStatus.UNAVAILABLE)
        self.assertEqual(result.terrain_type, "UNKNOWN")
        self.assertFalse(result.target_visible)

    def test_invalid_json_is_rejected(self) -> None:
        transport = FakeTransport({"message": {"content": "not json"}})
        result = VLMService(self.config(), _transport=transport).analyze(b"image")
        self.assertEqual(result.status, VLMStatus.INVALID_RESPONSE)

    def test_timeout_retries_then_degrades(self) -> None:
        transport = FakeTransport(TimeoutError(), TimeoutError())
        result = VLMService(self.config(), _transport=transport).analyze(b"image")
        self.assertEqual(result.status, VLMStatus.TIMEOUT)
        self.assertEqual(len(transport.calls), 2)

    def test_transient_failure_is_retried(self) -> None:
        transport = FakeTransport(OSError(), chat_response())
        result = VLMService(self.config(), _transport=transport).analyze(b"image")
        self.assertEqual(result.status, VLMStatus.SUCCESS)
        self.assertEqual(len(transport.calls), 2)

    def test_successful_strict_parsing_and_prompt_format(self) -> None:
        transport = FakeTransport(chat_response())
        service = VLMService(self.config(inference_timeout_s=17.0), _transport=transport)
        result = service.analyze(b"image")
        self.assertEqual(result.terrain_type, "snowfield")
        self.assertEqual(result.visible_landmarks, ("rock", "research station"))
        self.assertTrue(result.target_visible)
        self.assertEqual(result.scene_confidence, 0.91)
        path, payload, timeout = transport.calls[0]
        self.assertEqual(path, "/api/chat")
        self.assertEqual(timeout, 17.0)
        self.assertEqual(payload["messages"][0]["content"], SYSTEM_PROMPT)
        self.assertEqual(payload["options"]["temperature"], 0)
        self.assertFalse(payload["stream"])
        self.assertFalse(payload["format"]["additionalProperties"])

    def test_result_model_is_immutable(self) -> None:
        result = VLMService(
            self.config(),
            _transport=FakeTransport(chat_response()),
        ).analyze(b"image")
        with self.assertRaises(FrozenInstanceError):
            result.scene_summary = "changed"

    def test_is_available_uses_health_check(self) -> None:
        available = VLMService(
            self.config(),
            _transport=FakeTransport({"models": [{"model": "qwen2.5vl:7b"}]}),
        )
        unavailable = VLMService(
            self.config(),
            _transport=FakeTransport(OSError(), OSError()),
        )
        self.assertTrue(available.is_available())
        self.assertFalse(unavailable.is_available())

    def test_model_verification_detects_missing_model(self) -> None:
        service = VLMService(
            self.config(),
            _transport=FakeTransport({"models": [{"name": "other:latest"}]}),
        )
        self.assertFalse(service.verify_model())

    def test_extra_or_wrong_typed_fields_are_rejected(self) -> None:
        extra = dict(SCENE, conversational_reply="Certainly!")
        wrong_type = dict(SCENE, target_visible="yes")
        first = VLMService(
            self.config(),
            _transport=FakeTransport(chat_response(extra)),
        ).analyze(b"one")
        second = VLMService(
            self.config(),
            _transport=FakeTransport(chat_response(wrong_type)),
        ).analyze(b"two")
        self.assertEqual(first.status, VLMStatus.INVALID_RESPONSE)
        self.assertEqual(second.status, VLMStatus.INVALID_RESPONSE)

    def test_successful_results_are_cached(self) -> None:
        transport = FakeTransport(chat_response())
        config = replace(self.config(), cache_capacity=2, cache_ttl_s=5.0)
        service = VLMService(config, _transport=transport)
        first = service.analyze(b"same-image")
        second = service.analyze(b"same-image")
        self.assertIs(first, second)
        self.assertEqual(len(transport.calls), 1)

    def test_transport_usage_is_serialized_across_threads(self) -> None:
        transport = ConcurrentTransport()
        service = VLMService(self.config(), _transport=transport)
        results = []

        threads = [
            Thread(target=lambda image=image: results.append(service.analyze(image)))
            for image in (b"one", b"two")
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        self.assertEqual(transport.maximum_active, 1)
        self.assertEqual([item.status for item in results], [VLMStatus.SUCCESS] * 2)


if __name__ == "__main__":
    unittest.main()
