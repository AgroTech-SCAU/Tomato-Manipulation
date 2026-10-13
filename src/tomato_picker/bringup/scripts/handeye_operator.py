#!/usr/bin/env python3
"""Interactive operator commands for the dedicated hand-eye robot session"""

import json
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger


SERVICES = {
    "r": ("/handeye/recover_drag", "恢复拖拽"),
    "d": ("/handeye/drag/enable", "进入拖拽"),
    "c": ("/handeye/fault/compliant_recovery", "受限柔性脱困"),
    "h": ("/handeye/fault/rigid_hold", "故障刚性保持"),
    "f": ("/handeye/fault/clear", "仅清除故障"),
}


class HandeyeOperator(Node):
    def __init__(self):
        super().__init__("handeye_operator")
        # Never use Node's reserved _clients attribute for application state
        self._service_clients = {
            key: self.create_client(Trigger, name)
            for key, (name, _) in SERVICES.items()
        }
        self._last_status = None
        self._status_subscription = self.create_subscription(
            String, "/handeye/status", self._status, 10
        )

    def _status(self, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        short = (
            data.get("state"), data.get("running"),
            data.get("snapshot_valid"), data.get("fault_hold_mode"),
            data.get("drag_requested"), data.get("error"),
        )
        if short == self._last_status:
            return
        self._last_status = short
        state, running, valid, hold, drag, error = short
        print(
            f"\n状态 {state} | 工作线程 {'在线' if running else '停止'} | "
            f"位姿 {'有效' if valid else '无效'} | 保持 {hold} | "
            f"拖拽 {'已请求' if drag else '未请求'}",
            flush=True,
        )
        if error:
            print(f"故障 {error}", flush=True)

    def request(self, key):
        service_name, label = SERVICES[key]
        client = self._service_clients[key]
        if not client.wait_for_service(timeout_sec=1.5):
            print(f"无法连接 {service_name}", flush=True)
            return
        finished = threading.Event()
        response_holder = []
        future = client.call_async(Trigger.Request())

        def on_done(done):
            try:
                response_holder.append(done.result())
            except Exception as error:
                response_holder.append(error)
            finally:
                finished.set()

        future.add_done_callback(on_done)
        if not finished.wait(18.0):
            print(f"{label}未在等待时间内响应 | 请查看机器人状态", flush=True)
            return
        result = response_holder[0]
        if isinstance(result, Exception):
            print(f"{label}失败 | {result}", flush=True)
        else:
            print(f"{label} {'成功' if result.success else '失败'} | {result.message}", flush=True)


def main():
    # Let KeyboardInterrupt go through finally instead of tearing down the ROS context first
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
        print("手眼标定控制台 | r 恢复拖拽 | d 拖拽 | f 仅清故障 | c 受限柔性脱困 | h 故障刚性保持 | q 退出控制台")
        print("受限柔性脱困需经过 Core 的 Safety 许可 | c 脱困后先 h 再 r")
        print("退出控制台不会失能机械臂 | 请在 handeye.launch.py 终端执行 Ctrl+C 停放退出")
        while rclpy.ok():
            key = input("操作 > ").strip().lower()
            if key == "q":
                break
            if key in SERVICES:
                node.request(key)
            elif key:
                print("未知指令 | 输入 r d f c h 或 q")
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
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
