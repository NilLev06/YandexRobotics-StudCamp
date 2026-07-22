import json
import unittest
from unittest.mock import patch

import rover_dispatcher as rd


GOOD = [
    {"target": "gfsx_yolo", "action": "set_target_class", "class_name": "bottle"},
    {"target": "gfsx_robot", "action": "activate", "state": "start"},
]


class ValidationTests(unittest.TestCase):
    def test_accepts_exact_contract(self):
        self.assertEqual(rd.parse_and_validate_commands(json.dumps(GOOD), {"bottle"}), GOOD)

    def test_rejects_unknown_class(self):
        with self.assertRaises(rd.DispatcherError):
            rd.parse_and_validate_commands(json.dumps(GOOD), {"cup"})

    def test_rejects_changed_activation(self):
        bad = [GOOD[0], {**GOOD[1], "state": "stop"}]
        with self.assertRaises(rd.DispatcherError):
            rd.parse_and_validate_commands(json.dumps(bad), set())

    def test_rejects_extra_fields(self):
        bad = [{**GOOD[0], "url": "http://example.test"}, GOOD[1]]
        with self.assertRaises(rd.DispatcherError):
            rd.parse_and_validate_commands(json.dumps(bad), set())

    def test_extracts_nested_responses_output(self):
        response = {
            "output": [
                {"type": "reasoning", "content": None},
                {"content": [{"type": "output_text", "text": " ok "}]},
            ]
        }
        self.assertEqual(rd.extract_output_text(response), "ok")


class SafetyTests(unittest.TestCase):
    @patch.object(rd, "query_yandex", return_value=GOOD)
    @patch.object(rd, "capture_frame", return_value=b"jpeg")
    @patch.dict(
        rd.os.environ,
        {"YANDEX_FOLDER_ID": "folder", "YANDEX_API_KEY": "secret"},
        clear=True,
    )
    def test_default_is_dry_run(self, _capture, _query):
        with patch.object(rd, "send_to_unity") as sender:
            self.assertEqual(rd.main([]), 0)
            sender.assert_not_called()

    @patch.object(rd, "query_yandex", return_value=GOOD)
    @patch.object(rd, "capture_frame", return_value=b"jpeg")
    @patch.dict(
        rd.os.environ,
        {
            "YANDEX_FOLDER_ID": "folder",
            "YANDEX_API_KEY": "secret",
            "UNITY_API_URL": "http://unity:8088/api/m2m",
        },
        clear=True,
    )
    def test_send_requires_allowlist(self, _capture, _query):
        with self.assertRaises(rd.DispatcherError):
            rd.main(["--send"])


if __name__ == "__main__":
    unittest.main()
