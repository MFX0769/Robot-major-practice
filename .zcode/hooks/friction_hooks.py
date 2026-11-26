#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""friction_hooks.py — ZCode hook: friction-signal capture & context injection

/* EN · CN */
Capture friction signals (correction keywords / tool failures) per session,
inject reminders via hook additionalContext:
  user-prompt        match correction keywords in user prompt -> count + remind closure loop
  tool-failure       silent count on tool failure
  stop               at turn end, remind doc/lessons closure if unhandled friction
  session-compact    after context compaction, inject state-recovery instructions
  self-test          print sample outputs for manual verification (no state touched)

State: per-session JSON in %TEMP%/zcode_friction/ (git-clean, auto-scoped).
Log:   %TEMP%/zcode_friction/hook.log (one line per invocation, for debugging).
"""

import sys
import os
import re
import json
import time
import tempfile

# --- correction keywords: AGENTS.md "错误→规则闭环" trigger words + interrupt words ---
KEYWORDS = [
    "记下来", "这是教训", "写进规则", "下次别犯", "你错了", "规则缺了",
    "不对", "搞错了", "改错了", "谁让你", "我没让", "别乱", "不要乱",
    "停", "够了", "回滚", "撤销",
]

STATE_DIR = os.path.join(tempfile.gettempdir(), "zcode_friction")


def read_input():
    """Read hook stdin JSON; tolerate empty/invalid input. /* 读 hook stdin，容错 */"""
    try:
        raw = sys.stdin.read()
        if raw.strip():
            return json.loads(raw)
    except Exception:
        pass
    return {}


def session_id(inp):
    """Stable per-session key: env var -> stdin field -> sanitized. /* 会话标识 */"""
    sid = os.environ.get("CLAUDE_SESSION_ID") or os.environ.get("ZCODE_SESSION_ID") \
        or inp.get("session_id") or "adhoc"
    return re.sub(r"[^A-Za-z0-9_-]", "_", str(sid))[:80]


def load_state(sid):
    path = os.path.join(STATE_DIR, sid + ".json")
    try:
        with open(path, "r", encoding="utf-8") as f:
            st = json.load(f)
            st.setdefault("count", 0)
            st.setdefault("reminded", 0)
            st.setdefault("events", [])
            return st
    except Exception:
        return {"count": 0, "reminded": 0, "events": []}


def save_state(sid, st):
    try:
        os.makedirs(STATE_DIR, exist_ok=True)
        st["events"] = st["events"][-10:]  # keep tail only /* 只留尾部 */
        with open(os.path.join(STATE_DIR, sid + ".json"), "w", encoding="utf-8") as f:
            json.dump(st, f, ensure_ascii=False)
    except Exception:
        pass


def log_line(sid, event, note, emitted):
    """Append one invocation line; never raise. /* 调试日志，永不抛错 */"""
    try:
        os.makedirs(STATE_DIR, exist_ok=True)
        with open(os.path.join(STATE_DIR, "hook.log"), "a", encoding="utf-8") as f:
            ts = time.strftime("%Y-%m-%d %H:%M:%S")
            f.write("%s | %s | sid=%s | %s | emitted=%d\n" % (ts, event, sid, note, emitted))
    except Exception:
        pass


def emit(event, text):
    """Emit additionalContext JSON; strict schema, no extra keys. /* 严格键名 */"""
    out = {"hookSpecificOutput": {"hookEventName": event, "additionalContext": text}}
    sys.stdout.reconfigure(encoding="utf-8")
    print(json.dumps(out, ensure_ascii=False))


def cmd_user_prompt(inp):
    prompt = str(inp.get("prompt") or inp.get("user_prompt") or inp.get("message") or "")
    hits = [k for k in KEYWORDS if k in prompt]
    sid = session_id(inp)
    if not hits:
        log_line(sid, "UserPromptSubmit", "no-hit", 0)
        return 0
    st = load_state(sid)
    st["count"] += 1
    st["events"].append("prompt:" + ",".join(hits))
    save_state(sid, st)
    emit("UserPromptSubmit",
         "【摩擦信号·第%d次】用户消息命中纠偏关键词(%s)。执行错误→规则闭环："
         "①用自己的话复述教训，确认没曲解 ②分类：执行失误→立即纠正不改规则 / 规则缺漏→提补丁方案 "
         "③等用户确认再动手。先核对理解，别急着改代码。" % (st["count"], ",".join(hits)))
    log_line(sid, "UserPromptSubmit", "hit:" + ",".join(hits), 1)
    return 0


def cmd_tool_failure(inp):
    tool = str(inp.get("tool_name") or inp.get("tool") or inp.get("name") or "?")
    sid = session_id(inp)
    st = load_state(sid)
    st["count"] += 1
    st["events"].append("toolfail:" + tool)
    save_state(sid, st)
    log_line(sid, "PostToolUseFailure", "tool=" + tool, 0)
    return 0


def cmd_stop(inp):
    sid = session_id(inp)
    st = load_state(sid)
    if st["count"] > st["reminded"]:
        emit("Stop",
             "【摩擦信号·本会话%d次未处理】收尾自查(静默执行，均无则忽略)："
             "①改动文件的落盘三步(目录/README登记/索引)是否完成 ②教训是否该进 lessons.md 或 daily "
             "③有无说过'我会…'却未兑现的下一步。" % (st["count"] - st["reminded"]))
        st["reminded"] = st["count"]
        save_state(sid, st)
        log_line(sid, "Stop", "remind count=%d" % st["count"], 1)
    else:
        log_line(sid, "Stop", "clean", 0)
    return 0


def cmd_session_compact(inp):
    sid = session_id(inp)
    emit("SessionStart",
         "【压缩恢复】会话刚经历上下文压缩，摘要可能丢失：进行中任务的未落盘细节、关键实测数据、"
         "用户拍板项、承诺过的下一步。恢复动作：①读 develop_log/handoffs/ 最新一份 ②读 "
         "develop_log/daily_review.md 尾部 ③git status + git diff --stat 核对工作区 "
         "④向用户简报状态后再继续被压缩打断的任务。")
    log_line(sid, "SessionStart:compact", "injected", 1)
    return 0


def cmd_self_test(_inp=None):
    """Print sample outputs + simulate one hit cycle; no state files. /* 自检 */"""
    sample_prompt = {"session_id": "selftest", "prompt": "不对，这是教训，记下来"}
    print("== user-prompt hit output ==")
    emit("UserPromptSubmit", "(sample) 摩擦信号提醒文本")
    hits = [k for k in KEYWORDS if k in sample_prompt["prompt"]]
    print("== keyword match on sample: %s ==" % hits)
    print("== stop remind output ==")
    emit("Stop", "(sample) 收尾自查文本")
    print("== compact recovery output ==")
    emit("SessionStart", "(sample) 压缩恢复文本")
    return 0


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    inp = {} if cmd == "self-test" else read_input()
    handlers = {
        "user-prompt": cmd_user_prompt,
        "tool-failure": cmd_tool_failure,
        "stop": cmd_stop,
        "session-compact": cmd_session_compact,
        "self-test": cmd_self_test,
    }
    h = handlers.get(cmd)
    if h is None:
        return 0  # unknown subcommand: silent pass, never block session /* 未知命令静默放行 */
    return h(inp)


if __name__ == "__main__":
    sys.exit(main())
