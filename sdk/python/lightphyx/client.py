"""Thread-safe WebSocket RPC client for the light-3js-phyx server.

Control frames are JSON ``{id, method, params}`` / ``{id, result|error}``;
binary frames are ``[u32 LE request id][u8 channel][payload]``.
"""
from __future__ import annotations

import json
import socket
import struct
import threading
from typing import Any, Dict, List, Optional, Tuple

import websocket

CHANNEL_RGB = 1
CHANNEL_SEG = 2
CHANNEL_DEPTH = 3
CHANNEL_VIDEO = 4

Binaries = List[Tuple[int, memoryview]]


class RpcError(RuntimeError):
    pass


class _Pending:
    __slots__ = ("event", "result", "error", "binaries")

    def __init__(self) -> None:
        self.event = threading.Event()
        self.result: Any = None
        self.error: Optional[str] = None
        self.binaries: Binaries = []


class Client:
    def __init__(self, url: str, timeout: float = 120.0) -> None:
        self._ws = websocket.create_connection(url, enable_multithread=True)
        self._timeout = timeout
        self._lock = threading.Lock()
        self._next_id = 1
        self._pending: Dict[int, _Pending] = {}
        self._closed = False
        # Receive path: read frames straight off the socket with recv_into
        # (websocket-client's recv_data pulls 16 KB chunks and joins them —
        # ~80 syscalls plus a full copy per 1.2 MB pass). Server→client frames
        # are unmasked per RFC 6455, so this is a header parse and a bulk read.
        self._sock = self._ws.sock
        try:
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
            self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def _recv_exact(self, n: int) -> bytearray:
        buf = bytearray(n)
        view = memoryview(buf)
        got = 0
        while got < n:
            k = self._sock.recv_into(view[got:], n - got)
            if k == 0:
                raise ConnectionError("socket closed")
            got += k
        return buf

    def _read_frame(self) -> Tuple[int, bool, bytearray]:
        """One WebSocket frame → (opcode, fin, payload)."""
        head = self._recv_exact(2)
        b0, b1 = head[0], head[1]
        fin = bool(b0 & 0x80)
        opcode = b0 & 0x0F
        masked = bool(b1 & 0x80)
        length = b1 & 0x7F
        if length == 126:
            length = struct.unpack("!H", self._recv_exact(2))[0]
        elif length == 127:
            length = struct.unpack("!Q", self._recv_exact(8))[0]
        mask = self._recv_exact(4) if masked else None
        payload = self._recv_exact(length) if length else bytearray()
        if mask is not None:  # servers must not mask; tolerate it anyway
            for i in range(length):
                payload[i] ^= mask[i & 3]
        return opcode, fin, payload

    def _read_message(self) -> Tuple[int, bytearray]:
        """Reassemble a (possibly fragmented) message; handles control frames."""
        opcode, fin, payload = self._read_frame()
        while True:
            if opcode == websocket.ABNF.OPCODE_PING:
                self._ws.pong(bytes(payload))
            elif opcode == websocket.ABNF.OPCODE_PONG:
                pass
            elif opcode == websocket.ABNF.OPCODE_CLOSE:
                return opcode, payload
            else:
                break
            opcode, fin, payload = self._read_frame()
        while not fin:
            op2, fin, more = self._read_frame()
            if op2 == websocket.ABNF.OPCODE_PING:
                self._ws.pong(bytes(more))
                fin = False
                continue
            if op2 == websocket.ABNF.OPCODE_CLOSE:
                return op2, more
            payload += more
        return opcode, payload

    def _read_loop(self) -> None:
        try:
            while True:
                opcode, data = self._read_message()
                if opcode == websocket.ABNF.OPCODE_BINARY:
                    req_id, channel = struct.unpack_from("<IB", data, 0)
                    pending = self._pending.get(req_id)
                    if pending is not None:  # stray stream frames are dropped
                        # Zero-copy: keep a view; decoders wrap it with
                        # np.frombuffer (a 1–5 MB copy per pass otherwise).
                        pending.binaries.append((channel, memoryview(data)[5:]))
                elif opcode == websocket.ABNF.OPCODE_TEXT:
                    msg = json.loads(bytes(data))
                    pending = self._pending.get(msg.get("id"))
                    if pending is None:
                        continue
                    if "error" in msg and msg["error"]:
                        pending.error = msg["error"].get("message", "unknown error")
                    else:
                        pending.result = msg.get("result")
                    pending.event.set()
                elif opcode == websocket.ABNF.OPCODE_CLOSE:
                    break
        except Exception:
            pass
        finally:
            self._closed = True
            for pending in self._pending.values():
                pending.error = pending.error or "connection closed"
                pending.event.set()

    def _send(self, method: str, params: Dict[str, Any]) -> Tuple[int, _Pending]:
        if self._closed:
            raise RpcError("connection closed")
        with self._lock:
            req_id = self._next_id
            self._next_id += 1
            pending = _Pending()
            self._pending[req_id] = pending
            self._ws.send(json.dumps({"id": req_id, "method": method, "params": params}))
        return req_id, pending

    def _wait(self, req_id: int, pending: _Pending) -> Tuple[Any, Binaries]:
        if not pending.event.wait(self._timeout):
            self._pending.pop(req_id, None)
            raise RpcError(f"request {req_id} timed out")
        self._pending.pop(req_id, None)
        if pending.error is not None:
            raise RpcError(pending.error)
        return pending.result, pending.binaries

    def call(self, method: str, **params: Any) -> Any:
        result, _ = self._wait(*self._send(method, params))
        return result

    def call_binary(self, method: str, **params: Any) -> Tuple[Any, Binaries]:
        return self._wait(*self._send(method, params))

    def call_many(self, requests: List[Tuple[str, Dict[str, Any]]]) -> List[Tuple[Any, Binaries]]:
        """Issue several requests before waiting — pipelines across pool pages."""
        sent = [self._send(method, params) for method, params in requests]
        results: List[Tuple[Any, Binaries]] = []
        error: Optional[BaseException] = None
        # Drain EVERY pending entry even if one fails — a list comprehension
        # would short-circuit on the first error and leak the rest (their
        # pending entries + binary payloads never get removed).
        for req_id, pending in sent:
            try:
                results.append(self._wait(req_id, pending))
            except BaseException as exc:  # noqa: BLE001 — re-raised after draining
                self._pending.pop(req_id, None)
                if error is None:
                    error = exc
        if error is not None:
            raise error
        return results

    def close(self) -> None:
        try:
            self._ws.close()
        except Exception:
            pass
