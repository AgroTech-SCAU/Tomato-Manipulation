#!/usr/bin/env python3
"""Hand-eye operator with a stable terminal dashboard and safe service requests"""

import curses
import json
import os
import sys
import threading
import time
import unicodedata
from collections import deque

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger


SERVICES = {
    "r": ("/handeye/recover_drag", "恢复拖拽"),
    "d": ("/handeye/drag/enable", "进入拖拽"),
    "f": ("/handeye/fault/clear", "仅清故障"),
    "c": ("/handeye/fault/compliant_recovery", "受限柔性脱困"),
    "h": ("/handeye/fault/rigid_hold", "故障刚性保持"),
}

LABELS = {
    "COMPLIANT_DRAG": "柔性拖拽",
    "RIGID_HOLD": "刚性保持",
    "COMPLIANT_HOLD": "柔性保持",
    "RIGID_TRACKING": "刚性跟踪",
    "COMPLIANT_TRACKING": "柔性跟踪",
    "COMPLIANT_RECOVERY": "受限柔性恢复",
}


def mode_label(name):
    return LABELS.get(name, name or "未知")


def clip_line(value, max_columns):
    used = 0
    output = []
    for ch in str(value).replace("\n", " ").replace("\r", " "):
        size = 2 if unicodedata.east_asian_width(ch) in ("F", "W") else 1
        if used + size > max_columns:
            break
        output.append(ch)
        used += size
    return "".join(output)


class HandeyeOperator(Node):
    def __init__(self):
        super().__init__("handeye_operator")
        self._service_clients = {
            key: self.create_client(Trigger, name)
            for key, (name, _) in SERVICES.items()
        }
        self._status_lock = threading.Lock()
        self._latest_status = None
        self._last_received = 0.0
        self._messages = deque(maxlen=7)
        self._busy = False
        self._closed = threading.Event()
        self._status_subscription = self.create_subscription(
            String, "/handeye/status", self._on_status, 10
        )

    def _on_status(self, msg):
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(data, dict):
            return
        with self._status_lock:
            previous = self._latest_status or {}
            self._latest_status = data
            self._last_received = time.monotonic()
            # Keep the dashboard stable, record only meaningful state changes
            if data.get("state") != previous.get("state"):
                self._messages.appendleft("机械臂状态 " + str(data.get("state", "未知")))
            error = str(data.get("error") or "")
            if error and error != previous.get("error"):
                self._messages.appendleft("故障 " + error)

    def snapshot(self):
        with self._status_lock:
            return (
                dict(self._latest_status) if self._latest_status is not None else None,
                self._last_received,
                list(self._messages),
                self._busy,
            )

    def note(self, message):
        with self._status_lock:
            self._messages.appendleft(message)

    def request(self, key):
        service_name, label = SERVICES[key]
        client = self._service_clients[key]
        if not client.wait_for_service(timeout_sec=1.5):
            return f"{label}失败 | 服务未连接 {service_name}"
        future = client.call_async(Trigger.Request())
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(18.0):
            return f"{label}等待超时 | 请查看机械臂状态"
        try:
            response = future.result()
        except Exception as error:
            return f"{label}失败 | {error}"
        return f"{label}{'成功' if response.success else '失败'} | {response.message}"

    def submit(self, key):
        with self._status_lock:
            if self._busy:
                self._messages.appendleft("已有操作正在执行 | 请等待结果")
                return
            self._busy = True
            self._messages.appendleft("已发送 " + SERVICES[key][1] + " 请求")

        def task():
            try:
                result = self.request(key)
                if not self._closed.is_set():
                    self.note(result)
            except Exception as error:
                if not self._closed.is_set():
                    self.note("操作异常 " + str(error))
            finally:
                with self._status_lock:
                    self._busy = False

        threading.Thread(target=task, name="handeye-operator-command", daemon=True).start()


def describe(data, last_received):
    if data is None:
        return ["状态  等待 /handeye/status", "请先启动 handeye.launch.py"]
    if time.monotonic() - last_received > 2.0:
        return ["状态  与机械臂状态发布端断开", "最后状态已过期 | 禁止依据旧状态继续操作"]

    state = data.get("state", "未知")
    running = bool(data.get("running"))
    valid = bool(data.get("pose_ready", data.get("snapshot_valid", False)))
    requested = data.get("requested_mode") or ("COMPLIANT_DRAG" if data.get("drag_requested") else "未请求拖拽")
    observed = data.get("observed_command_mode") or "UNKNOWN"
    lines = [
        f"机械臂  {state}     工作线程 {'在线' if running else '停止'}     位姿 {'可采集' if valid else '不可采集'}",
        f"控制请求  {mode_label(requested)}",
    ]
    if state == "FAULT":
        lines.append("故障保持  " + mode_label(data.get("fault_hold_mode")))
        lines.append("控制命令  故障期间不报告正常阻抗模式")
    elif state == "ACTIVE":
        if observed in ("UNKNOWN", "AMBIGUOUS"):
            lines.append("控制命令  暂未能从 Core 当前 kp/kd 确认模式")
        else:
            lines.append("控制命令  " + mode_label(observed) + "  [由当前周期增益识别]")
        if requested == "COMPLIANT_DRAG" and observed != "COMPLIANT_DRAG":
            lines.append("提示  拖拽切换已提交，正在等待控制周期反馈或检查模式增益")
    else:
        lines.append("控制命令  不可用")
    if data.get("error"):
        lines.append("故障原因  " + str(data["error"]))
    return lines


def draw(stdscr, node):
    stdscr.erase()
    height, width = stdscr.getmaxyx()
    def line(y, content, attr=0):
        if 0 <= y < height and width > 2:
            try:
                stdscr.addstr(y, 1, clip_line(content, width - 2), attr)
            except curses.error:
                pass

    if height < 15 or width < 52:
        line(0, "终端窗口过小，请放大后操作")
        line(2, "q 退出   r 恢复拖拽   c 受限脱困")
        stdscr.refresh()
        return

    data, received, events, busy = node.snapshot()
    line(1, "手眼标定  |  机器人操作台", curses.A_BOLD)
    line(2, "r 恢复拖拽   d 进入拖拽   f 仅清故障   c 受限脱困   h 刚性保持   q 退出")
    line(3, "指令直接按单键，无需回车 | q 仅退出控制台，不关闭机械臂")
    line(4, "─" * min((width - 2) // 2, 65))
    summary = describe(data, received)
    for i, msg in enumerate(summary[:6]):
        line(5 + i, msg, curses.A_BOLD if i == 0 else 0)
    line(11, "─" * min((width - 2) // 2, 65))
    line(12, "操作结果" + ("  [执行中]" if busy else "  [就绪]"), curses.A_BOLD)
    for i, entry in enumerate(events[:max(0, height - 15)]):
        line(13 + i, entry)
    line(height - 2, "注意 关节越限应先脱离危险位置 | Ctrl+C 停放请到 launch 终端")
    stdscr.refresh()


def terminal_ui(stdscr, node):
    try:
        curses.curs_set(0)
    except curses.error:
        pass
    stdscr.timeout(180)
    while rclpy.ok():
        draw(stdscr, node)
        value = stdscr.getch()
        if value < 0:
            continue
        if value in (ord("q"), ord("Q"), 27):
            return
        key = chr(value).lower() if value <= 127 else ""
        if key in SERVICES:
            node.submit(key)


def fallback_ui(node):
    print("手眼标定控制台 | r 恢复拖拽 | d 拖拽 | f 清故障 | c 受限脱困 | h 刚性保持 | s 状态 | q 退出", flush=True)
    print("模式状态只在按 s 时输出，不会打断输入", flush=True)
    while rclpy.ok():
        try:
            key = input("操作 > ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break
        if key == "q":
            break
        if key == "s":
            data, seen, _, _ = node.snapshot()
            print("\n".join(describe(data, seen)), flush=True)
        elif key in SERVICES:
            # Show the result in the foreground rather than printing asynchronously
            print(node.request(key), flush=True)
        elif key:
            print("未知指令", flush=True)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    executor = None
    worker = None
    try:
        node = HandeyeOperator()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        worker = threading.Thread(target=executor.spin, name="handeye-operator-spin", daemon=True)
        worker.start()
        if sys.stdin.isatty() and sys.stdout.isatty() and os.environ.get("TERM") not in (None, "", "dumb"):
            try:
                curses.wrapper(terminal_ui, node)
            except curses.error:
                print("终端不支持固定界面，已切换到普通控制台", flush=True)
                fallback_ui(node)
        else:
            fallback_ui(node)
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        if node is not None:
            node._closed.set()
        if executor is not None:
            executor.shutdown(timeout_sec=3.0)
        if worker is not None:
            worker.join(timeout=3.0)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
