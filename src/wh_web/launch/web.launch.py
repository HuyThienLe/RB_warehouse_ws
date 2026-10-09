#!/usr/bin/env python3
"""web.launch.py – Khởi động Web Server cục bộ phục vụ quản lý kho (Bài 8.5)."""

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    web_server_proc = ExecuteProcess(
        cmd=["uvicorn", "wh_web.app:app", "--host", "0.0.0.0", "--port", "8000"],
        output="screen",
    )

    return LaunchDescription([
        web_server_proc,
    ])
