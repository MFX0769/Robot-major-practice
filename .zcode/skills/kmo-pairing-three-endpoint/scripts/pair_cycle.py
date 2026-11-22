"""Bench pairing cycles: Android App, BK3633 bridge and remote.

No flash, OTA, motor or calibration commands. The remote trigger is sent only
after the selected bridge UART actually reports App command 0x00E0.
"""

import datetime as dt
import argparse
import hashlib
import json
import pathlib
import re
import shutil
import subprocess
import threading
import time
import xml.etree.ElementTree as ET

import serial
from serial.tools import list_ports

TRIGGER = bytes.fromhex("AA 55 AA 00 00 A9 F5 BB")
# 2026-11-19: the remote UART misses frames while the app sleeps (seen in rounds
# 2/4 of 20261119_0c4c_run02). Wake bytes are skipped by the frame parser.
WAKE = bytes(8)
WAKE_GAP_S = 0.05
TRIGGER_CONFIRM_S = 0.8
TRIGGER_MAX_ATTEMPTS = 3
# [P3 2026-11-22] app mode: bridge serial unavailable (COM124 BLE adv broken; App
# rides a serial-less third bridge). Arm on App begin tap + periodic retrigger,
# verified by App UI + remote-side UART markers instead of bridge 0x00E0.
TRIGGER_MODE = "bridge"
APP_TRIGGER_MAX_ATTEMPTS = 40
APP_TRIGGER_GAP_S = 1.8
TRIGGER_ECHO = "[CMD] pair triggered by uart"
ROOT = None
PORTS = ()
ADB_SERIAL = None
ADB_BIN = "adb"
APP_PACKAGE = "com.kamingo.bike"
BRIDGE_NAME = "kmo_bridge_t"
LAST_ROUND = 10


def now():
    return dt.datetime.now().isoformat(timespec="milliseconds")


def _port_tag(name):
    return re.sub(r"[^A-Za-z0-9_.-]", "_", name)


def adb(*args, timeout=12):
    result = subprocess.run(
        [ADB_BIN, "-s", ADB_SERIAL, *args], capture_output=True, timeout=timeout, check=True
    )
    return result.stdout


def ui_nodes():
    adb("shell", "uiautomator", "dump", "/sdcard/kmo_pair_auto.xml")
    xml = adb("exec-out", "cat", "/sdcard/kmo_pair_auto.xml")
    return [node.attrib for node in ET.fromstring(xml).iter("node")]


def label(node):
    return node.get("text", "") + "\n" + node.get("content-desc", "")


def find(nodes, text, exact=False):
    hits = [n for n in nodes if label(n).strip() == text] if exact else []
    if not hits:
        hits = [n for n in nodes if text in label(n)]
    if not hits:
        return None
    return min(hits, key=lambda n: area(n.get("bounds", "")))


def find_exact(nodes, text):
    hits = [n for n in nodes if label(n).strip() == text]
    return min(hits, key=lambda n: area(n.get("bounds", ""))) if hits else None


def area(bounds):
    match = re.fullmatch(r"\[(\d+),(\d+)\]\[(\d+),(\d+)\]", bounds)
    if not match:
        return 10**9
    x1, y1, x2, y2 = map(int, match.groups())
    return max(0, x2 - x1) * max(0, y2 - y1)


def tap(node):
    match = re.fullmatch(r"\[(\d+),(\d+)\]\[(\d+),(\d+)\]", node["bounds"])
    if not match:
        raise RuntimeError(f"No tap bounds: {node}")
    x1, y1, x2, y2 = map(int, match.groups())
    adb("shell", "input", "tap", str((x1 + x2) // 2), str((y1 + y2) // 2))


def wait_ui(text, timeout=20):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        nodes = ui_nodes()
        hit = find(nodes, text)
        if hit:
            return nodes, hit
        time.sleep(0.25)
    raise TimeoutError(f"App UI missing {text!r}")


class Capture:
    def __init__(self, tag):
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.events = []
        self.active_round = 0
        self.armed = False
        self.trigger_time = None
        self.trigger_attempts = 0
        self.trigger_confirmed = threading.Event()
        self.remote_paired = threading.Event()
        self.trigger_thread = None
        self.ports = {}
        self.files = {}
        try:
            for name in PORTS:
                port = serial.Serial()
                port.port = name
                port.baudrate = 1_000_000
                port.timeout = 0.03
                port.rts = False
                port.dtr = False
                port.open()
                self.ports[name] = port
                port_tag = _port_tag(name)
                self.files[name] = (ROOT / f"{tag}_{port_tag}.bin").open("wb")
        except Exception:
            for port in self.ports.values():
                port.close()
            for file in self.files.values():
                file.close()
            raise
        self.timeline = (ROOT / f"{tag}_timeline.jsonl").open("w", encoding="utf-8")
        self.threads = [threading.Thread(target=self.reader, args=(n,), daemon=True)
                        for n in PORTS]
        for thread in self.threads:
            thread.start()

    def reader(self, name):
        line = bytearray()
        while not self.stop.is_set():
            data = self.ports[name].read(4096)
            if not data:
                continue
            self.files[name].write(data)
            self.files[name].flush()
            for byte in data:
                if byte in (10, 13):
                    if line:
                        self.record(name, line.decode("gb18030", errors="replace"))
                    line.clear()
                elif len(line) < 4096:
                    line.append(byte)

    def record(self, name, line):
        with self.lock:
            event = {"at": now(), "round": self.active_round,
                     "port": name, "line": line}
            self.events.append(event)
            self.timeline.write(json.dumps(event, ensure_ascii=False) + "\n")
            self.timeline.flush()
            if name == PORTS[1] and TRIGGER_ECHO in line:
                self.trigger_confirmed.set()
            if name == PORTS[1] and "Pairing Process Success" in line:
                self.remote_paired.set()
            if name == PORTS[0] and self.armed and "app_cfg_rx: cmd=0x00E0" in line:
                self._trigger_remote_locked("BRIDGE_E0")
            important = ("Pairing Process Success", "Pairing FAILED", "PAIR_OK",
                         "[BIND]", "app_cfg_rx: cmd=0x00E0", "rf=1 tgt=1")
            if any(word in line for word in important):
                print(f"{event['at']} {name}: {line[:200]}", flush=True)

    def _marker_locked(self, text):
        marker = {"at": now(), "round": self.active_round, "port": "HOST", "line": text}
        self.events.append(marker)
        self.timeline.write(json.dumps(marker, ensure_ascii=False) + "\n")
        self.timeline.flush()
        return marker["at"]

    def _trigger_remote_locked(self, source):
        if not self.armed:
            return
        self.armed = False
        self.trigger_time = now()
        self.trigger_attempts = 0
        self.trigger_confirmed.clear()
        self.trigger_thread = threading.Thread(
            target=self._trigger_worker, args=(source, self.active_round), daemon=True)
        self.trigger_thread.start()

    def _trigger_worker(self, source, round_no):
        """Wake the remote UART, send the trigger, resend only while no echo arrived."""
        port = self.ports[PORTS[1]]
        max_att = APP_TRIGGER_MAX_ATTEMPTS if TRIGGER_MODE == "app" else TRIGGER_MAX_ATTEMPTS
        for attempt in range(1, max_att + 1):
            if self.trigger_confirmed.is_set():
                if TRIGGER_MODE != "app":
                    break
                # app mode: echo means remote got it; give the handshake time,
                # then keep retriggering until the remote reports pairing done.
                if self.remote_paired.wait(6.0):
                    break
                if self.active_round != round_no or not self.armed:
                    break
                self.trigger_confirmed.clear()
            if self.active_round != round_no or not self.armed:
                break
            port.write(WAKE)
            time.sleep(WAKE_GAP_S)
            port.write(TRIGGER)
            with self.lock:
                self.trigger_attempts = attempt
                at = self._marker_locked(
                    f"REMOTE_TRIGGER source={source} attempt={attempt}")
            print(f"{at} round {round_no}: {source} -> remote UART trigger #{attempt}", flush=True)
            if self.trigger_confirmed.wait(TRIGGER_CONFIRM_S):
                if TRIGGER_MODE != "app":
                    break
                if self.remote_paired.wait(6.0):
                    break
                self.trigger_confirmed.clear()
                if self.active_round != round_no or not self.armed:
                    break
                time.sleep(APP_TRIGGER_GAP_S)
        with self.lock:
            self._marker_locked(
                f"REMOTE_TRIGGER_DONE attempts={self.trigger_attempts} "
                f"confirmed={int(self.trigger_confirmed.is_set())}")

    def trigger_remote(self, source):
        with self.lock:
            self._trigger_remote_locked(source)

    def slice(self, start):
        with self.lock:
            return list(self.events[start:])

    def close(self):
        self.stop.set()
        for thread in self.threads:
            thread.join(timeout=1)
        for port in self.ports.values():
            port.close()
        for file in self.files.values():
            file.close()
        self.timeline.close()


def ensure_detail():
    nodes = ui_nodes()
    if find(nodes, "已连接设备"):
        return nodes
    bridge = find(nodes, BRIDGE_NAME)
    if not bridge:
        raise RuntimeError(f"App is neither home with {BRIDGE_NAME} nor device detail")
    tap(bridge)
    return wait_ui("设备详情")[0]


def unbind_if_needed():
    nodes = ui_nodes()
    def remote_page(items):
        return (find_exact(items, "解绑") and not find(items, "已连接设备")
                and any(("控制器 " in label(n) or label(n).strip() == "kmo_remote")
                        for n in items))
    if not remote_page(nodes):
        nodes = ensure_detail()
        ready_deadline = time.monotonic() + 18
        while True:
            card = next((n for n in nodes if ("kmo_remote" in label(n) or "控制器 " in label(n))
                         and "解绑" in label(n)), None)
            if card:
                break
            if find(nodes, "添加设备\n控制器"):
                return "already_unbound"
            if time.monotonic() >= ready_deadline:
                raise RuntimeError("Cannot identify controller binding card")
            time.sleep(0.5)
            nodes = ui_nodes()
        for attempt in range(1, 4):
            tap(card)
            print(f"{now()} REMOTE_CARD_TAP #{attempt} {card.get('bounds')}", flush=True)
            deadline = time.monotonic() + 7
            while time.monotonic() < deadline:
                nodes = ui_nodes()
                if remote_page(nodes):
                    break
            if remote_page(nodes):
                break
            card = next((n for n in nodes if ("kmo_remote" in label(n) or "控制器 " in label(n))
                         and "解绑" in label(n)), None)
            if not card:
                raise RuntimeError("Remote card disappeared before opening its detail")
        else:
            raise TimeoutError("Remote detail page not reached after three taps")
    button = find_exact(nodes, "解绑")
    tap(button)
    _, confirm = wait_ui("确认", 10)
    tap(confirm)
    wait_ui("添加设备\n控制器", 20)
    return "unbound"


def pair_round(cap, number):
    cap.active_round = number
    cap.trigger_time = None
    cap.trigger_attempts = 0
    start = len(cap.events)
    print(f"{now()} ROUND_START {number}/{LAST_ROUND}", flush=True)
    unbind_state = unbind_if_needed()
    print(f"{now()} ROUND_UNBIND {number}: {unbind_state}", flush=True)
    nodes, add = wait_ui("添加设备\n控制器", 10)
    tap(add)
    try:
        _, begin = wait_ui("开始绑定", 7)
    except TimeoutError:
        nodes = ui_nodes()
        add = find(nodes, "添加设备\n控制器")
        if add:
            tap(add)
        _, begin = wait_ui("开始绑定", 12)
    cap.armed = True
    tap(begin)
    print(f"{now()} ROUND_APP_BEGIN {number}", flush=True)
    if TRIGGER_MODE == "app":
        cap.trigger_remote("APP_BEGIN")
    deadline = time.monotonic() + 85
    last_begin_tap = time.monotonic()
    begin_retries = 0
    app_success = False
    address = None
    while time.monotonic() < deadline:
        nodes = ui_nodes()
        notice = find(nodes, "知道了")
        if notice:
            tap(notice)
            continue
        begin_again = next((n for n in nodes if label(n).strip() == "开始绑定"), None)
        if (not cap.trigger_time and begin_again
                and time.monotonic() - last_begin_tap >= 5 and begin_retries < 2):
            tap(begin_again)
            begin_retries += 1
            last_begin_tap = time.monotonic()
            print(f"{now()} ROUND_APP_BEGIN_RETRY {number} #{begin_retries}", flush=True)
            continue
        if find(nodes, "遥控器绑定成功"):
            app_success = True
            for n in nodes:
                match = re.search(r"控制器 ([0-9A-F]{2}(?::[0-9A-F]{2}){4})", label(n))
                if match:
                    address = match.group(1)
                    break
            break
        if find(nodes, "遥控器绑定失败"):
            break
        time.sleep(0.25)
    cap.armed = False
    if app_success:
        _, done = wait_ui("已完成", 5)
        tap(done)
        wait_ui(BRIDGE_NAME, 20)
        bind_deadline = time.monotonic() + 12
        while time.monotonic() < bind_deadline:
            if any("[BIND] APP bind complete" in event["line"]
                   for event in cap.slice(start)):
                break
            time.sleep(0.2)
    evidence = cap.slice(start)
    lines = [event["line"] for event in evidence]
    checks = {
        "app_success": app_success,
        "bridge_request": any("app_cfg_rx: cmd=0x00E0" in x for x in lines),
        "remote_cmd_confirmed": any(TRIGGER_ECHO in x for x in lines),
        "bridge_success": any("Slave: Pairing Process Success Completed." in x for x in lines),
        "remote_success": any("Host: Pairing Process Success Completed." in x for x in lines),
        "ble_window": any("[BIND] window open" in x for x in lines),
        "remote_ble_mode": any("rf=1 tgt=1" in x for x in lines),
        "app_bind_complete": any("[BIND] APP bind complete" in x for x in lines),
    }
    result = {"round": number, "at": now(), "unbind": unbind_state,
              "remote_trigger_at": cap.trigger_time,
              "remote_trigger_attempts": cap.trigger_attempts, "address": address,
              "checks": checks, "events": len(evidence)}
    (ROOT / f"round_{number:02d}.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print("ROUND_RESULT " + json.dumps(result, ensure_ascii=False), flush=True)
    # The remote UART can interleave two printf calls in one line.  BLE mode
    # plus APP bind complete proves the window was active even if its text line
    # is garbled. Keep ble_window as an independent observability field.
    if TRIGGER_MODE == "app":
        required = ("app_success", "remote_cmd_confirmed", "remote_success",
                    "remote_ble_mode")
    else:
        required = ("app_success", "bridge_request", "bridge_success", "remote_success",
                    "remote_ble_mode", "app_bind_complete")
    if not all(checks[key] for key in required):
        try:
            for _ in range(4):
                nodes = ui_nodes()
                if find(nodes, BRIDGE_NAME):
                    break
                later = find(nodes, "稍后绑定")
                if later:
                    tap(later)
                else:
                    adb("shell", "input", "keyevent", "4")
                time.sleep(1)
            wait_ui(BRIDGE_NAME, 12)
        except Exception as exc:
            raise RuntimeError(f"Round {number} failed and App recovery failed: {exc}") from exc
    return result


def _port_identity(name, expected_serial, role):
    ports = list(list_ports.comports())
    if name:
        matches = [p for p in ports if p.device.casefold() == name.casefold()]
    elif expected_serial:
        matches = [p for p in ports if p.serial_number == expected_serial]
    else:
        raise RuntimeError(f"{role}: provide a USB serial or an explicit port")
    if len(matches) != 1:
        raise RuntimeError(f"{role}: expected one matching port, found {len(matches)}")
    p = matches[0]
    if expected_serial and p.serial_number != expected_serial:
        raise RuntimeError(
            f"{role}: {p.device} USB serial {p.serial_number!r} != {expected_serial!r}"
        )
    return {"port": p.device, "description": p.description,
            "vid": p.vid, "pid": p.pid, "serial": p.serial_number, "hwid": p.hwid}


def _sha(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main():
    global ROOT, PORTS, ADB_SERIAL, ADB_BIN, APP_PACKAGE, BRIDGE_NAME, LAST_ROUND, TRIGGER_MODE
    parser = argparse.ArgumentParser()
    parser.add_argument("--bench", type=pathlib.Path,
                        help="JSON bench identity file; no COM numbers required")
    parser.add_argument("--bridge-port")
    parser.add_argument("--remote-port")
    parser.add_argument("--expect-bridge-serial")
    parser.add_argument("--expect-remote-serial")
    parser.add_argument("--adb-serial")
    parser.add_argument("--adb", help="ADB executable; defaults to PATH")
    parser.add_argument("--app-package")
    parser.add_argument("--bridge-name")
    parser.add_argument("--out", type=pathlib.Path)
    parser.add_argument("--preflight", action="store_true",
                        help="check host dependencies and identities without opening ports or changing binding")
    parser.add_argument("--start-round", type=int, default=1)
    parser.add_argument("--last-round", type=int)
    parser.add_argument("--rounds", type=int, help="number of rounds from --start-round")
    parser.add_argument("--trigger-mode", choices=("bridge", "app"), default="bridge",
                        help="bridge: arm trigger on bridge UART 0x00E0 (default); "
                             "app: arm on App begin tap + periodic retrigger "
                             "(bridge serial absent; verify via App UI + remote UART)")
    args = parser.parse_args()
    TRIGGER_MODE = args.trigger_mode
    if args.rounds is not None and args.last_round is not None:
        parser.error("Use --rounds or --last-round, not both")
    LAST_ROUND = (args.start_round + args.rounds - 1 if args.rounds is not None
                  else args.last_round if args.last_round is not None else 10)
    if not (1 <= args.start_round <= LAST_ROUND):
        parser.error("Require 1 <= start-round <= last-round")
    bench = json.loads(args.bench.read_text(encoding="utf-8")) if args.bench else {}
    if not isinstance(bench, dict):
        parser.error("Bench file must contain a JSON object")
    bridge_serial = args.expect_bridge_serial or bench.get("bridge_usb_serial")
    remote_serial = args.expect_remote_serial or bench.get("remote_usb_serial")
    identities = [
        _port_identity(args.bridge_port, bridge_serial, "bridge"),
        _port_identity(args.remote_port, remote_serial, "remote"),
    ]
    PORTS = tuple(identity["port"] for identity in identities)
    if PORTS[0].casefold() == PORTS[1].casefold():
        parser.error("Bridge and remote ports must differ")
    APP_PACKAGE = args.app_package or bench.get("app_package") or APP_PACKAGE
    BRIDGE_NAME = args.bridge_name or bench.get("bridge_name") or BRIDGE_NAME
    ADB_BIN = shutil.which(args.adb or bench.get("adb") or "adb")
    if not ADB_BIN:
        raise RuntimeError("adb not found; install Android platform-tools or pass --adb")
    devices = subprocess.run([ADB_BIN, "devices"], capture_output=True,
                             check=True, timeout=12).stdout.decode("utf-8", errors="replace")
    connected = [line.split()[0] for line in devices.splitlines()[1:]
                 if line.strip().endswith("device")]
    ADB_SERIAL = args.adb_serial or bench.get("adb_serial") or (
        connected[0] if len(connected) == 1 else None)
    if not ADB_SERIAL or ADB_SERIAL not in connected:
        raise RuntimeError(f"Select one connected ADB device: {devices}")
    pid = adb("shell", "pidof", APP_PACKAGE).decode().strip()
    if not pid.isdigit():
        raise RuntimeError(f"App {APP_PACKAGE} not running")
    if args.preflight:
        print(json.dumps({"adb_serial": ADB_SERIAL, "app_package": APP_PACKAGE,
                          "app_pid": pid, "bridge_name": BRIDGE_NAME,
                          "ports": identities}, ensure_ascii=False, indent=2))
        return
    if args.out is None:
        parser.error("--out is required for a pairing run")
    ROOT = args.out.resolve()
    ROOT.mkdir(parents=True, exist_ok=False)
    (ROOT / "identities.json").write_text(json.dumps({
        "captured_at": now(), "adb_serial": ADB_SERIAL,
        "app_package": APP_PACKAGE, "app_pid": pid,
        "bridge_name": BRIDGE_NAME, "ports": identities,
        "bench_sha256": _sha(args.bench) if args.bench else None,
        "trigger_frame_hex": TRIGGER.hex(),
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    tag = (f"rounds_{args.start_round:02d}_{LAST_ROUND:02d}_"
           f"{dt.datetime.now().strftime('%H%M%S')}")
    log_file = (ROOT / f"{tag}_app_logcat.bin").open("wb")
    log_proc = subprocess.Popen(
        [ADB_BIN, "-s", ADB_SERIAL, "logcat", f"--pid={pid}", "-v", "threadtime", "-T", "1"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT
    )
    try:
        cap = Capture(tag)
    except Exception:
        log_proc.terminate()
        log_proc.wait(timeout=5)
        log_file.close()
        raise
    def log_reader():
        for line in iter(log_proc.stdout.readline, b""):
            log_file.write(line)
            log_file.flush()
            # A phone 0x0160 may come from a different bridge. Only the
            # selected bridge's UART 0x00E0 can arm the remote trigger.
    log_thread = threading.Thread(target=log_reader, daemon=True)
    log_thread.start()
    try:
        for number in range(args.start_round, LAST_ROUND + 1):
            pair_round(cap, number)
    finally:
        cap.armed = False
        cap.close()
        log_proc.terminate()
        log_proc.wait(timeout=5)
        log_thread.join(timeout=2)
        log_file.close()
        paths = sorted(p for p in ROOT.iterdir() if p.is_file() and p.name != "SHA256SUMS.txt")
        (ROOT / "SHA256SUMS.txt").write_text(
            "".join(f"{_sha(p)}  {p.name}\n" for p in paths), encoding="utf-8"
        )


if __name__ == "__main__":
    main()
