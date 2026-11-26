#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""workflow_hooks.py — ZCode hook: session context injection + deterministic guards

/* EN · CN */
Modes:
  session-start    SessionStart(startup/resume/clear): inject branch/git-status/latest handoff/weekly INDEX pointers
  pre-tool-use     PreToolUse(Write|Edit|Bash): deterministic guards, exit 2 = block
                     guard A  daily_review.md append-only: block Write/Edit overwrite (bash `cat >>` still allowed)
                     guard B  output/ & develop_log/ deletion + `git clean`: block (bins must be kept in git)
  self-test        print decisions for sample inputs; no state touched

Output contract mirrors friction_hooks.py: strict hookSpecificOutput JSON on stdout (proven in-session),
blocking via exit code 2 + stderr. Debug log: %TEMP%/zcode_friction/hook.log (shared with friction hook).
"""

import sys
import os
import re
import json
import subprocess
import time
import tempfile

STATE_DIR = os.path.join(tempfile.gettempdir(), "zcode_friction")

# Guard B patterns: deletion verbs targeting protected dirs, or any `git clean`.
# Kept narrow on purpose (调研报告 方案二2c: 白名单从窄起走，避免误拦).
RE_GIT_CLEAN = re.compile(r"\bgit\s+clean\b")
RE_DELETE_PROTECTED = re.compile(
    r"\b(rm|rmdir|del|rd|erase|Remove-Item)\b[^|;&>]*\b(output|develop_log)\b", re.IGNORECASE)


def read_input():
    """Read hook stdin JSON; tolerate empty/invalid input. /* 读 stdin，容错 */"""
    try:
        raw = sys.stdin.read()
        if raw.strip():
            return json.loads(raw)
    except Exception:
        pass
    return {}


def log_line(event, note, emitted):
    """One debug line per invocation; never raise. /* 调试日志，永不抛错 */"""
    try:
        os.makedirs(STATE_DIR, exist_ok=True)
        with open(os.path.join(STATE_DIR, "hook.log"), "a", encoding="utf-8") as f:
            ts = time.strftime("%Y-%m-%d %H:%M:%S")
            f.write("%s | %s | %s | emitted=%d\n" % (ts, event, note, emitted))
    except Exception:
        pass


def emit(event, text):
    """Emit additionalContext JSON (same schema as friction_hooks). /* 严格键名 */"""
    out = {"hookSpecificOutput": {"hookEventName": event, "additionalContext": text}}
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    print(json.dumps(out, ensure_ascii=False))


def deny(reason):
    """Block a tool call: message on stderr, exit 2. /* 阻断=exit2+stderr */"""
    log_line("PreToolUse", "DENY: " + reason[:80], 1)
    try:
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass
    sys.stderr.write("【workflow_hooks 守门拦截】%s\n" % reason)
    return 2


def project_dir(inp):
    return os.environ.get("ZCODE_PROJECT_DIR") or inp.get("cwd") or os.getcwd()


def run_cmd(args, cwd, timeout=4):
    """Run a quick read-only command, return stdout or '' on any failure. /* 只读快查，失败返回空 */"""
    try:
        r = subprocess.run(args, cwd=cwd, capture_output=True, text=True,
                           timeout=timeout, shell=False, encoding="utf-8", errors="replace")
        return (r.stdout or "").strip()
    except Exception:
        return ""


def cmd_session_start(inp):
    """Inject startup context: branch/status/handoff/weekly pointers. /* 注入启动四件套 */
    [H1/H2/H3 09-27] twin-repo newest-handoff lookup, post-compaction ritual,
    missing wrap-up warning. All extra logic fail-safe: any error degrades to
    the original four-line injection (hook must never raise)."""
    cwd = project_dir(inp)
    branch = run_cmd(["git", "branch", "--show-current"], cwd)
    st = run_cmd(["git", "status", "--porcelain"], cwd)
    st_lines = [l for l in st.splitlines() if l.strip()]
    st_summary = "%d 个改动文件" % len(st_lines)
    handoff = ""
    handoff_src = ""
    try:
        cands = []
        for base in (cwd,
                     "D:/Kamingo/bk3633_main_my/SDK/projects/rc_remote",
                     "D:/Kamingo/bk3633_main_my/SDK_fhss_exp/projects/rc_remote"):
            ho = run_cmd(["bash", "-c", "ls -t develop_log/handoffs/ 2>/dev/null | head -1"], base)
            if ho:
                name = ho.splitlines()[0].strip()
                full = os.path.join(base.replace("/", os.sep), "develop_log", "handoffs", name)
                if os.path.isfile(full):
                    cands.append((os.path.getmtime(full), base, name))
        if cands:
            cands.sort(reverse=True)
            _, handoff_src, handoff = cands[0]
    except Exception:
        handoff, handoff_src = "", ""
    if not handoff:
        ho = run_cmd(["bash", "-c", "ls -t develop_log/handoffs/ 2>/dev/null | head -1"], cwd)
        if ho:
            handoff = ho.splitlines()[0].strip()
    weekly = ""
    wk = run_cmd(["bash", "-c", "ls -t develop_log/plans/weekly/*.md 2>/dev/null | head -1"], cwd)
    if wk:
        weekly = os.path.basename(wk.splitlines()[0].strip())
    wrapup_warn = ""
    try:
        dl = run_cmd(["bash", "-c", "tail -40 develop_log/daily_review.md 2>/dev/null"], cwd)
        if dl and ("收尾快照" not in dl) and ("pre-compact" not in dl.lower()):
            wrapup_warn = " ⚠️上次会话未见收尾快照标记=压缩丢失风险高，先补读 handoff。"
    except Exception:
        pass
    today = time.strftime("%Y-%m-%d")
    ritual = ("【压缩后仪式】①先读最新 handoff 全文 ②再读计划文件当前刀序(规格§11/weekly) "
              "③向用户复述三行计划，确认后再动手。")
    src = str(inp.get("source") or "")
    src_note = "（本次由压缩触发）" if "compact" in src else ""
    emit("SessionStart",
         "【会话上下文·自动注入】日期=%s 分支=%s 工作区=%s。"
         "①最新 handoff=develop_log/handoffs/%s（读它接续上下文；无则跳过）"
         "②当前周文件=develop_log/plans/weekly/%s（待续/已裁决在此）"
         "③跨会话知识先查 develop_log/knowledge/INDEX.md"
         "④开发日志=develop_log/daily_review.md（看尾部）。"
         "启动规则详见 AGENTS.md「会话启动」节。"
         "%s%s%s"
         % (today, branch or "?", st_summary,
            handoff or "（空）", weekly or "（空）",
            ("（handoff 来自 %s）" % handoff_src) if handoff_src and "SDK" not in cwd.replace("\\", "/") else "",
            src_note, ritual + wrapup_warn))
    log_line("SessionStart", "startup injected branch=%s st=%d src=%s" % (branch, len(st_lines), src or "?"), 1)
    return 0


def cmd_pre_tool_use(inp):
    """Guards: A=daily append-only, B=output//develop_log deletion + git clean. /* 双守门 */"""
    tool = str(inp.get("tool_name") or inp.get("tool") or "")
    ti = inp.get("tool_input") or {}
    file_path = str(ti.get("file_path") or ti.get("notebook_path") or "").replace("\\", "/")
    command = str(ti.get("command") or "")

    # Guard A: daily_review.md is append-only (AGENTS.md hard rule #8) /* 守门A */
    if tool in ("Write", "Edit") and file_path.endswith("develop_log/daily_review.md"):
        return deny("develop_log/daily_review.md 为 append-only 日志，禁止 Write/Edit 覆写或改写。"
                    "追加请用 bash：`cat >> develop_log/daily_review.md << 'EOF' ... EOF`（先 date 取实测时间戳）。")
    # Guard B: protected dirs / git clean (AGENTS.md hard rule #7) /* 守门B */
    if tool == "Bash":
        if RE_GIT_CLEAN.search(command):
            return deny("禁止 git clean：output/ 下 bin 是编译产物留存铁律保护的固件副本，"
                        "develop_log/ 为项目记忆。确需清理时请用户手动执行并逐项核对。")
        if RE_DELETE_PROTECTED.search(command):
            return deny("禁止删除 output/ 或 develop_log/ 下的内容（编译产物留存 + 项目记忆）。"
                        "若为误判请调整命令措辞并向用户说明。")
    log_line("PreToolUse", "pass tool=%s" % tool, 0)
    return 0


def cmd_self_test(_inp=None):
    """Simulate guard decisions; no state touched. /* 自检 */"""
    cases = [
        ("session-start", {}),
        ("Write daily", {"tool_name": "Write",
                         "tool_input": {"file_path": "D:/x/develop_log/daily_review.md"}}),
        ("Edit daily", {"tool_name": "Edit",
                        "tool_input": {"file_path": "D:\\x\\develop_log\\daily_review.md"}}),
        ("git clean", {"tool_name": "Bash", "tool_input": {"command": "git clean -fd output/"}}),
        ("rm output", {"tool_name": "Bash", "tool_input": {"command": "rm -rf output/app"}}),
        ("del output", {"tool_name": "Bash", "tool_input": {"command": "cmd /c del output\\a.bin"}}),
        ("pass: normal rm", {"tool_name": "Bash", "tool_input": {"command": "rm build/tmp.o"}}),
        ("pass: read output", {"tool_name": "Bash", "tool_input": {"command": "ls output/app"}}),
        ("pass: append", {"tool_name": "Bash",
                          "tool_input": {"command": "cat >> develop_log/daily_review.md"}}),
    ]
    for name, case in cases:
        if name == "session-start":
            print("== session-start sample ==")
            cmd_session_start(case)
            continue
        # suppress deny stderr noise in self-test: capture decision by code path
        rc = cmd_pre_tool_use(case)
        print("case %-18s -> exit=%d (%s)" % (name, rc, "BLOCK" if rc else "pass"))
    return 0


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    inp = {} if cmd == "self-test" else read_input()
    handlers = {
        "session-start": cmd_session_start,
        "pre-tool-use": cmd_pre_tool_use,
        "self-test": cmd_self_test,
    }
    h = handlers.get(cmd)
    if h is None:
        return 0  # unknown subcommand: silent pass, never block session /* 未知命令静默放行 */
    return h(inp)


if __name__ == "__main__":
    sys.exit(main())
