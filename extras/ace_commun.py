# ace_commun.py

import os
import json
import struct
import select
import socket
import logging
import serial
from queue import Queue

# Константы протокола
FRAME_START_1 = 0xFF
FRAME_START_2 = 0xAA
FRAME_END = 0xFE
MIN_FRAME_SIZE = 7  # 2 start + 2 len + 2 CRC + 1 end

# Ошибки
RESPOND_TIMEOUT_ERROR = "Respond timeout with the ACE PRO"
UNABLE_TO_COMMUN_ERROR = "Unable to communicate with the ACE PRO"
OPEN_REMOTE_DEV_ERROR = "Unable to open remote dev"
OPEN_SERIAL_DEV_ERROR = "Unable to open serial port"
NOT_FOUND_SERIAL_ERROR = "Not found serial port"

def _calc_crc(data):
    crc = 0xFFFF
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp ^= (tmp & 0x0F) << 4
        crc = ((tmp << 8) | (crc >> 8)) ^ (tmp >> 4) ^ ((tmp & 0xFF) << 3)
    return crc & 0xFFFF

class AceCommun:
    def __init__(self, name, baud):
        self.name = name
        self.baud = baud
        self.is_connected = False
        self.dev = None
        self.send_time = 0.0
        self.read_buffer = b""
        self.request_id = 0
        self.callback_map = {}
        self.send_queue = Queue()

    def check_port_exists(self, path):
        return os.path.exists(path) and not os.path.isdir(path)

    def connect(self):
        try:
            if self.name.startswith("tcp@"):
                host_port = self.name[4:].split(':', 1)
                host = host_port[0]
                port = int(host_port[1]) if len(host_port) > 1 else 8888
                sock = socket.create_connection((host, port), timeout=5)
                sock.setblocking(False)
                self.dev = sock
            else:
                if not self.check_port_exists(self.name):
                    raise OSError(f"{NOT_FOUND_SERIAL_ERROR} {self.name}")
                self.dev = serial.Serial(
                    self.name, self.baud,
                    timeout=0,  # non-blocking
                    write_timeout=0
                )
                self.dev.flushInput()
            self.is_connected = True
            self.send_queue = Queue()
            self.request_id = 0
            self.callback_map = {}
            return None
        except Exception as e:
            logging.exception("ACE connect failed")
            return e

    def disconnect(self):
        if self.dev:
            try:
                self.dev.close()
            except:
                pass
        self.is_connected = False
        self.dev = None
        self.read_buffer = b""
        self.send_time = 0.0

    def get_fd(self):
        if hasattr(self.dev, 'fileno'):
            return self.dev.fileno()
        return -1

    def _send_request(self, req):
        req_id = req.get("id")
        if req_id is None:
            req["id"] = self.request_id
            self.request_id += 1

        payload = json.dumps(req, separators=(',', ':')).encode()
        length = len(payload)
        crc = _calc_crc(payload)

        frame = bytearray()
        frame.append(FRAME_START_1)
        frame.append(FRAME_START_2)
        frame.append(length & 0xFF)
        frame.append((length >> 8) & 0xFF)
        frame.extend(payload)
        frame.append(crc & 0xFF)
        frame.append((crc >> 8) & 0xFF)
        frame.append(FRAME_END)

        if hasattr(self.dev, 'send'):
            self.dev.send(frame)
        else:
            self.dev.write(frame)
            self.dev.flush()

    def writer(self, eventtime):
        if not self.send_queue.empty():
            task = self.send_queue.get_nowait()
            req, callback = task
            req_id = self.request_id
            self.request_id += 1
            self.callback_map[req_id] = callback
            req["id"] = req_id
            self._send_request(req)
            self.send_time = eventtime

    def reader(self, eventtime):
        if not self.dev:
            return UNABLE_TO_COMMUN_ERROR

        try:
            ready, _, _ = select.select([self.dev], [], [], 0)
            if not ready:
                if eventtime - self.send_time > 2.0:
                    return RESPOND_TIMEOUT_ERROR
                return None

            if hasattr(self.dev, 'recv'):
                raw = self.dev.recv(4096)
            else:
                raw = self.dev.read(4096)

            if not raw:
                if eventtime - self.send_time > 2.0:
                    return RESPOND_TIMEOUT_ERROR
                return None

            self.read_buffer += raw
            while True:
                idx = self.read_buffer.find(bytes([FRAME_END]))
                if idx == -1:
                    break
                frame = self.read_buffer[:idx+1]
                self.read_buffer = self.read_buffer[idx+1:]

                if len(frame) < MIN_FRAME_SIZE:
                    continue
                if frame[0] != FRAME_START_1 or frame[1] != FRAME_START_2:
                    logging.warning("Invalid ACE frame header")
                    continue
                if frame[-1] != FRAME_END:
                    continue

                length = frame[2] | (frame[3] << 8)
                expected_len = 4 + length + 2 + 1
                if len(frame) < expected_len:
                    continue

                payload = frame[4:4+length]
                crc_received = frame[4+length] | (frame[4+length+1] << 8)
                crc_calculated = _calc_crc(payload)

                if crc_received != crc_calculated:
                    logging.warning("ACE CRC mismatch")
                    continue

                try:
                    resp = json.loads(payload.decode())
                    req_id = int(resp.get("id", -1))
                    if req_id in self.callback_map:
                        cb = self.callback_map.pop(req_id)
                        cb(resp)
                except Exception as e:
                    logging.exception("ACE JSON parse error")

        except Exception as e:
            logging.exception("ACE read error")
            return f"{UNABLE_TO_COMMUN_ERROR}: {e}"

        return None

    def push_send_queue(self, request, callback):
        self.send_queue.put_nowait((request, callback))

    def is_send_queue_empty(self):
        return self.send_queue.empty()