"""Allow-listed child process management for mapping and navigation stacks."""
from __future__ import annotations

import os
import shlex
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .utils import EventJournal


@dataclass
class ManagedProcess:
    name: str
    command: str
    process: subprocess.Popen
    log_path: str
    started_at: float


class ProcessManager:
    """Starts only commands configured by ROS parameters, never commands from HTTP payloads."""

    def __init__(self, journal: EventJournal, log_root: str):
        self._journal = journal
        self._log_root = Path(os.path.expanduser(log_root)).resolve()
        self._log_root.mkdir(parents=True, exist_ok=True)
        self._items: dict[str, ManagedProcess] = {}
        self._lock = threading.RLock()

    def start(self, name: str, command_template: str, substitutions: dict[str, str] | None = None) -> dict[str, Any]:
        substitutions = substitutions or {}
        if not command_template.strip():
            return {"success": False, "message": f"{name} command is not configured"}

        with self._lock:
            existing = self._items.get(name)
            if existing and existing.process.poll() is None:
                return {
                    "success": True,
                    "message": f"{name} is already running",
                    "pid": existing.process.pid,
                    "already_running": True,
                }

            try:
                command = command_template.format(**substitutions)
            except KeyError as exc:
                return {"success": False, "message": f"missing command placeholder: {exc}"}

            try:
                argv = shlex.split(command)
            except ValueError as exc:
                return {"success": False, "message": f"invalid configured command: {exc}"}
            if not argv:
                return {"success": False, "message": "configured command is empty"}

            timestamp = time.strftime("%Y%m%d_%H%M%S")
            log_path = self._log_root / f"{name}_{timestamp}.log"
            log_file = open(log_path, "ab", buffering=0)
            try:
                process = subprocess.Popen(
                    argv,
                    stdin=subprocess.DEVNULL,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                    env=os.environ.copy(),
                    close_fds=True,
                )
            except Exception as exc:
                log_file.close()
                return {"success": False, "message": f"failed to start {name}: {exc}"}

            self._items[name] = ManagedProcess(
                name=name,
                command=command,
                process=process,
                log_path=str(log_path),
                started_at=time.time(),
            )
            self._journal.add(f"Started {name} process", "success", "process", pid=process.pid)
            return {
                "success": True,
                "message": f"{name} started",
                "pid": process.pid,
                "log_path": str(log_path),
            }

    def stop(self, name: str, timeout_s: float = 8.0) -> dict[str, Any]:
        with self._lock:
            item = self._items.get(name)
            if not item or item.process.poll() is not None:
                return {"success": True, "message": f"{name} is not running", "already_stopped": True}
            process = item.process

        def send(sig: int) -> None:
            try:
                os.killpg(os.getpgid(process.pid), sig)
            except ProcessLookupError:
                pass

        send(signal.SIGINT)
        try:
            process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            send(signal.SIGTERM)
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                send(signal.SIGKILL)
                process.wait(timeout=2.0)

        code = process.poll()
        self._journal.add(f"Stopped {name} process", "warning", "process", exit_code=code)
        return {"success": True, "message": f"{name} stopped", "exit_code": code}

    def status(self) -> dict[str, dict[str, Any]]:
        result: dict[str, dict[str, Any]] = {}
        with self._lock:
            items = list(self._items.items())
        for name, item in items:
            code = item.process.poll()
            result[name] = {
                "running": code is None,
                "pid": item.process.pid,
                "exit_code": code,
                "command": item.command,
                "log_path": item.log_path,
                "started_at": item.started_at,
                "uptime_s": round(max(0.0, time.time() - item.started_at), 1) if code is None else 0.0,
            }
        return result

    def stop_all(self) -> None:
        with self._lock:
            names = list(self._items.keys())
        for name in names:
            try:
                self.stop(name, timeout_s=3.0)
            except Exception:
                pass
