"""Offline regression for the bridge-gated remote trigger."""

import io
import json
import pathlib
import threading
import tempfile
import unittest
from types import SimpleNamespace
from unittest import mock

import pair_cycle


class FakePort:
    def __init__(self, on_trigger=None):
        self.writes = []
        self.on_trigger = on_trigger

    def write(self, data):
        self.writes.append(data)
        if data == pair_cycle.TRIGGER and self.on_trigger:
            self.on_trigger()


def make_capture(remote_port):
    pair_cycle.PORTS = ("COM3", "COM4")
    capture = pair_cycle.Capture.__new__(pair_cycle.Capture)
    capture.lock = threading.Lock()
    capture.events = []
    capture.active_round = 1
    capture.armed = True
    capture.trigger_time = None
    capture.trigger_attempts = 0
    capture.trigger_confirmed = threading.Event()
    capture.trigger_thread = None
    capture.timeline = io.StringIO()
    capture.ports = {"COM3": FakePort(), "COM4": remote_port}
    return capture


class TriggerTest(unittest.TestCase):
    def setUp(self):
        self._saved = (pair_cycle.WAKE_GAP_S, pair_cycle.TRIGGER_CONFIRM_S)
        pair_cycle.WAKE_GAP_S = 0.0
        pair_cycle.TRIGGER_CONFIRM_S = 0.05

    def tearDown(self):
        pair_cycle.WAKE_GAP_S, pair_cycle.TRIGGER_CONFIRM_S = self._saved

    def test_only_selected_bridge_e0_arms_once(self):
        capture = make_capture(FakePort())
        capture.record("APP", "RX RSP_TRIGGER_24G_PAIR 0x0160")
        capture.record("COM4", "app_cfg_rx: cmd=0x00E0")
        self.assertIsNone(capture.trigger_thread)
        self.assertEqual(capture.ports["COM4"].writes, [])

        capture.record("COM3", "app_cfg_rx: cmd=0x00E0")
        capture.record("COM3", "app_cfg_rx: cmd=0x00E0")
        capture.trigger_thread.join(2)
        self.assertFalse(capture.armed)
        self.assertEqual(capture.ports["COM3"].writes, [])

    def test_no_echo_resends_bounded_with_wake(self):
        capture = make_capture(FakePort())
        capture.record("COM3", "app_cfg_rx: cmd=0x00E0")
        capture.trigger_thread.join(2)
        expected = [pair_cycle.WAKE, pair_cycle.TRIGGER] * pair_cycle.TRIGGER_MAX_ATTEMPTS
        self.assertEqual(capture.ports["COM4"].writes, expected)
        self.assertEqual(capture.trigger_attempts, pair_cycle.TRIGGER_MAX_ATTEMPTS)
        self.assertTrue(any("confirmed=0" in e["line"] for e in capture.events))

    def test_echo_stops_after_first_attempt(self):
        holder = {}
        port = FakePort(on_trigger=lambda: holder["cap"].record(
            "COM4", pair_cycle.TRIGGER_ECHO))
        capture = make_capture(port)
        holder["cap"] = capture
        capture.record("COM3", "app_cfg_rx: cmd=0x00E0")
        capture.trigger_thread.join(2)
        self.assertEqual(port.writes, [pair_cycle.WAKE, pair_cycle.TRIGGER])
        self.assertEqual(capture.trigger_attempts, 1)
        self.assertTrue(any("confirmed=1" in e["line"] for e in capture.events))

    def test_echo_from_bridge_port_does_not_confirm(self):
        capture = make_capture(FakePort())
        capture.record("COM3", "app_cfg_rx: cmd=0x00E0")
        capture.record("COM3", pair_cycle.TRIGGER_ECHO)
        capture.trigger_thread.join(2)
        self.assertEqual(capture.trigger_attempts, pair_cycle.TRIGGER_MAX_ATTEMPTS)


class PortableBenchTest(unittest.TestCase):
    @staticmethod
    def port(device, serial):
        return SimpleNamespace(device=device, serial_number=serial,
                               description="USB UART", vid=0x1234, pid=0x5678,
                               hwid="USB VID:PID=1234:5678")

    def test_auto_discovery_uses_usb_serial_not_com_number(self):
        ports = [self.port("COM19", "remote-a"), self.port("COM82", "bridge-a")]
        with mock.patch.object(pair_cycle.list_ports, "comports", return_value=ports):
            bridge = pair_cycle._port_identity(None, "bridge-a", "bridge")
            remote = pair_cycle._port_identity(None, "remote-a", "remote")
        self.assertEqual((bridge["port"], remote["port"]), ("COM82", "COM19"))

    def test_ambiguous_or_wrong_serial_stops_before_opening_port(self):
        ports = [self.port("COM19", "same"), self.port("COM82", "same")]
        with mock.patch.object(pair_cycle.list_ports, "comports", return_value=ports):
            with self.assertRaisesRegex(RuntimeError, "found 2"):
                pair_cycle._port_identity(None, "same", "bridge")
            with self.assertRaisesRegex(RuntimeError, "!= 'different'"):
                pair_cycle._port_identity("COM19", "different", "bridge")

    def test_posix_port_name_is_safe_evidence_filename(self):
        self.assertEqual(pair_cycle._port_tag("/dev/ttyUSB0"), "_dev_ttyUSB0")

    def test_preflight_uses_bench_file_without_opening_serial_or_writing_evidence(self):
        with tempfile.TemporaryDirectory() as tmp:
            cfg = pathlib.Path(tmp) / "bench.json"
            cfg.write_text(json.dumps({"bridge_usb_serial": "bridge-a",
                                       "remote_usb_serial": "remote-a",
                                       "app_package": "org.example.bike"}), encoding="utf-8")
            ports = [self.port("/dev/ttyUSB2", "bridge-a"),
                     self.port("/dev/ttyUSB0", "remote-a")]
            adb_devices = SimpleNamespace(stdout=b"List of devices attached\nphone-1\tdevice\n")
            with mock.patch.object(pair_cycle.list_ports, "comports", return_value=ports), \
                 mock.patch.object(pair_cycle.shutil, "which", return_value="/usr/bin/adb"), \
                 mock.patch.object(pair_cycle.subprocess, "run", return_value=adb_devices) as run, \
                 mock.patch.object(pair_cycle, "adb", return_value=b"1234"), \
                 mock.patch.object(pair_cycle.serial, "Serial") as serial_open, \
                 mock.patch("sys.argv", ["pair_cycle.py", "--bench", str(cfg), "--preflight"]):
                pair_cycle.main()
            self.assertEqual(run.call_args.args[0], ["/usr/bin/adb", "devices"])
            serial_open.assert_not_called()
            self.assertEqual(pair_cycle.PORTS, ("/dev/ttyUSB2", "/dev/ttyUSB0"))
            self.assertFalse((pathlib.Path(tmp) / "evidence").exists())


if __name__ == "__main__":
    unittest.main()
