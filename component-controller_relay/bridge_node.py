#!/usr/bin/env python3
"""Bidirectional relay between two rclpy processes running different RMW
implementations.

RMW_IMPLEMENTATION is selected once per process (it's resolved via dynamic
library loading at rclpy/rclcpp init and can't be changed or mixed within a
process), so a single node can never publish on Fast DDS and Zenoh at the
same time. This script runs as two containers instead - one built against
rmw_fastrtps_cpp (the DDS domain micro_ros_agent bridges the microcontroller
onto) and one against rmw_zenoh_cpp (everything else on this rover) - linked
by a local TCP loopback connection. Each side deserializes/reserializes using
the real message class, so the receiving side is a genuine typed
publisher/service and shows up correctly for `ros2 topic echo` and
`ros2 service call`, unlike a raw DDS<->Zenoh protocol bridge (rmw_zenoh does
not speak the same wire format as those - its key expressions embed a
REP-2016 type hash that generic DDS bridges don't produce, which is why
topics show up in `topic list` but `echo` never receives anything).

Topics
------
BRIDGE_TOPICS drives which topics get bridged and in which direction, shared
identically by both containers:

    BRIDGE_TOPICS="/topic/a:pkg/msg/TypeA:up,/topic/b:pkg/msg/TypeB:down"

  - "up"   : microcontroller (DDS) -> rest of the rover (Zenoh)
  - "down" : rest of the rover (Zenoh) -> microcontroller (DDS)

Services
--------
BRIDGE_SERVICES works the same way, but "direction" describes which side
*originates the call*, not which side owns the real server - the real server
always lives on the opposite side from the caller:

    BRIDGE_SERVICES="/svc/a:pkg/srv/TypeA:down"

  - "down" : caller on Zenoh, real server on DDS (e.g. rapha_teleop calling
             into the microcontroller) - this side creates a fake `Service`
             that intercepts real calls and relays them; the DDS side holds a
             real `Client` to the actual server.
  - "up"   : caller on DDS, real server on Zenoh (mirror image).

Each process (BRIDGE_SIDE=dds or BRIDGE_SIDE=zenoh) only acts on the entries
relevant to its own side; see the controller_relay_dds / controller_relay_zenoh
services in docker-compose.yml.
"""
import importlib
import os
import queue
import random
import socket
import struct
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message

SIDE = os.environ.get("BRIDGE_SIDE", "unknown")
SOCKET_ROLE = os.environ.get("BRIDGE_SOCKET_ROLE", "server")


def _local(dds_name, zenoh_name):
    """The topic/service name to use on THIS process's local ROS graph. The
    Zenoh name is always the shared wire key both sides route frames by."""
    return dds_name if SIDE == "dds" else zenoh_name


SOCKET_HOST = os.environ.get("BRIDGE_SOCKET_HOST", "127.0.0.1")
SOCKET_PORT = int(os.environ.get("BRIDGE_SOCKET_PORT", "9999"))
SERVICE_CALL_TIMEOUT_SEC = float(os.environ.get("BRIDGE_SERVICE_TIMEOUT_SEC", "10.0"))

DEFAULT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# The source side subscribes best-effort, which receives both reliable and
# best-effort publishers (the firmware IMU is best-effort). The sink side
# republishes reliable, which every downstream subscriber accepts.
SOURCE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

FRAME_TOPIC = 0
FRAME_SVC_REQUEST = 1
FRAME_SVC_RESPONSE = 2

# 1 byte kind + 8 byte call_id (0 for topic frames) + 2 byte name length + 4 byte payload length
FRAME_HEADER = "!BQHI"
FRAME_HEADER_LEN = struct.calcsize(FRAME_HEADER)


def log(*args):
    print(f"[relay:{SIDE}]", *args, flush=True)


def import_ros_type(type_name):
    """'pkg/msg/Type' or 'pkg/srv/Type' -> the actual generated class."""
    pkg, kind, name = type_name.split("/")
    return getattr(importlib.import_module(f"{pkg}.{kind}"), name)


def parse_entries(raw):
    """'/name:pkg/msg/Type:up,...' -> [(dds_name, zenoh_name, type_name, direction), ...]

    The name field may optionally rename the topic/service between the two
    sides using 'dds_name=zenoh_name' - the microcontroller (DDS) side uses
    dds_name locally, the rest of the rover (Zenoh) side uses zenoh_name. This
    lets us give firmware topics whose hardcoded names are inconsistent (e.g.
    '/joint_states', '/controller/imu') clean, uniform '/controller/*' names on
    the Zenoh side without touching the firmware. With no '=', both sides use
    the same name (backwards compatible)."""
    entries = []
    for item in raw.split(","):
        item = item.strip()
        if not item:
            continue
        parts = item.split(":")
        if len(parts) != 3:
            log(f"WARNING: ignoring malformed entry '{item}' "
                f"(expected [dds=]name:pkg/kind/Type:up|down)")
            continue
        name, type_name, direction = parts
        if "=" in name:
            dds_name, zenoh_name = name.split("=", 1)
        else:
            dds_name = zenoh_name = name
        if direction not in ("up", "down"):
            log(f"WARNING: ignoring entry '{item}' with unknown direction '{direction}'")
            continue
        entries.append((dds_name, zenoh_name, type_name, direction))
    return entries


def send_frame(sock, kind, call_id, name, payload):
    name_b = name.encode("utf-8")
    header = struct.pack(FRAME_HEADER, kind, call_id, len(name_b), len(payload))
    sock.sendall(header + name_b + payload)


def recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("peer closed connection")
        buf.extend(chunk)
    return bytes(buf)


def recv_frame(sock):
    header = recv_exact(sock, FRAME_HEADER_LEN)
    kind, call_id, name_len, payload_len = struct.unpack(FRAME_HEADER, header)
    name = recv_exact(sock, name_len).decode("utf-8")
    payload = recv_exact(sock, payload_len)
    return kind, call_id, name, payload


class SocketLink:
    """Local loopback connection to the peer relay process. Reconnects
    automatically if the peer container restarts. Received frames are
    dispatched synchronously (from the reader thread) to on_frame - keep
    that fast/non-blocking (it is: dict lookup + publish, or dict lookup +
    call_async/event.set, never a blocking round trip)."""

    def __init__(self, role, host, port, on_frame):
        self.role = role
        self.host = host
        self.port = port
        self.on_frame = on_frame
        self.outgoing = queue.Queue(maxsize=2000)
        self._sock = None
        self._sock_lock = threading.Lock()
        self._listener = None
        # One writer thread for the link's lifetime; it always sends on the
        # current socket, across reconnects.
        threading.Thread(target=self._connection_loop, daemon=True).start()
        threading.Thread(target=self._writer_loop, daemon=True).start()

    def _accept_server(self):
        if self._listener is None:
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((self.host, self.port))
            srv.listen(1)
            self._listener = srv
        log(f"waiting for peer relay on {self.host}:{self.port}...")
        conn, addr = self._listener.accept()
        log(f"peer connected from {addr}")
        return conn

    def _connect_client(self):
        while True:
            try:
                sock = socket.create_connection((self.host, self.port), timeout=2.0)
                log(f"connected to peer relay at {self.host}:{self.port}")
                return sock
            except OSError:
                time.sleep(1.0)

    def _connection_loop(self):
        while True:
            try:
                sock = (
                    self._accept_server()
                    if self.role == "server"
                    else self._connect_client()
                )
            except Exception as exc:  # noqa: BLE001
                log(f"connection setup failed: {exc}, retrying...")
                time.sleep(1.0)
                continue

            # Blocking reads: idle periods between commands are normal, and a
            # dead peer on loopback is detected at once.
            sock.settimeout(None)
            with self._sock_lock:
                self._sock = sock
            try:
                while True:
                    kind, call_id, name, payload = recv_frame(sock)
                    self.on_frame(kind, call_id, name, payload)
            except Exception as exc:  # noqa: BLE001
                log(f"connection lost: {exc}")
            finally:
                with self._sock_lock:
                    if self._sock is sock:
                        self._sock = None
                try:
                    sock.close()
                except OSError:
                    pass
            # loop back around and (re)connect/accept again

    def _writer_loop(self):
        while True:
            kind, call_id, name, payload = self.outgoing.get()
            with self._sock_lock:
                sock = self._sock
            if sock is None:
                continue  # no active connection; drop until reconnected
            try:
                send_frame(sock, kind, call_id, name, payload)
            except OSError as exc:
                log(f"send failed for {name}: {exc}, dropping until reconnect")

    def send(self, kind, call_id, name, payload):
        try:
            self.outgoing.put_nowait((kind, call_id, name, payload))
        except queue.Full:
            log(f"outgoing queue full, dropping frame for {name}")


class RelayNode(Node):
    def __init__(self, link, topic_sources, topic_sinks, service_real_clients, service_fake_servers):
        super().__init__(f"controller_relay_{SIDE}")
        self.link = link
        self._topic_group = ReentrantCallbackGroup()
        self._sinks = {}  # topic -> (publisher, msg_class)
        self._real_clients = {}  # service name -> (client, srv_type)
        self._svc_lock = threading.Lock()
        self._pending_calls = {}  # call_id -> (threading.Event, dict holder)

        for dds_name, zenoh_name, type_name, _direction in topic_sources:
            self._add_topic_source(_local(dds_name, zenoh_name), zenoh_name, type_name)
        for dds_name, zenoh_name, type_name, _direction in topic_sinks:
            self._add_topic_sink(_local(dds_name, zenoh_name), zenoh_name, type_name)
        for dds_name, zenoh_name, type_name, _direction in service_real_clients:
            self._add_real_client(_local(dds_name, zenoh_name), zenoh_name, type_name)
        for dds_name, zenoh_name, type_name, _direction in service_fake_servers:
            self._add_fake_server(_local(dds_name, zenoh_name), zenoh_name, type_name)

    # -- topics --------------------------------------------------------

    def _add_topic_source(self, local, wire, type_name):
        try:
            msg_class = import_ros_type(type_name)
        except (ImportError, AttributeError, ValueError) as exc:
            log(f"cannot bridge topic {local}: failed to import {type_name}: {exc}")
            return

        def _cb(msg, _wire=wire):
            self.link.send(FRAME_TOPIC, 0, _wire, serialize_message(msg))

        self.create_subscription(msg_class, local, _cb, SOURCE_QOS, callback_group=self._topic_group)
        rename = "" if local == wire else f" as {wire}"
        log(f"forwarding topic {local} ({type_name}) -> peer{rename}")

    def _add_topic_sink(self, local, wire, type_name):
        try:
            msg_class = import_ros_type(type_name)
        except (ImportError, AttributeError, ValueError) as exc:
            log(f"cannot bridge topic {local}: failed to import {type_name}: {exc}")
            return
        pub = self.create_publisher(msg_class, local, DEFAULT_QOS)
        self._sinks[wire] = (pub, msg_class)
        rename = "" if local == wire else f" (peer sends {wire})"
        log(f"relaying topic {local} ({type_name}) <- peer{rename}")

    # -- services --------------------------------------------------------

    def _add_real_client(self, local, wire, type_name):
        """This side has the real service; forward relayed requests to it."""
        try:
            srv_type = import_ros_type(type_name)
        except (ImportError, AttributeError, ValueError) as exc:
            log(f"cannot bridge service {local}: failed to import {type_name}: {exc}")
            return
        client = self.create_client(srv_type, local, callback_group=self._topic_group)
        self._real_clients[wire] = (client, srv_type)
        log(f"forwarding service calls for {local} ({type_name}) to the real server here")

    def _add_fake_server(self, local, wire, type_name):
        """The real service lives on the peer; intercept local callers here."""
        try:
            srv_type = import_ros_type(type_name)
        except (ImportError, AttributeError, ValueError) as exc:
            log(f"cannot bridge service {local}: failed to import {type_name}: {exc}")
            return

        group = MutuallyExclusiveCallbackGroup()

        def _cb(request, response, _wire=wire):
            call_id = random.getrandbits(63)
            event = threading.Event()
            holder = {}
            with self._svc_lock:
                self._pending_calls[call_id] = (event, holder)
            self.link.send(FRAME_SVC_REQUEST, call_id, _wire, serialize_message(request))

            got = event.wait(timeout=SERVICE_CALL_TIMEOUT_SEC)
            with self._svc_lock:
                self._pending_calls.pop(call_id, None)

            if not got:
                log(f"WARNING: service call to {_wire} timed out after "
                    f"{SERVICE_CALL_TIMEOUT_SEC}s (peer relay or real server unreachable)")
                return response

            try:
                return deserialize_message(holder["payload"], srv_type.Response)
            except Exception as exc:  # noqa: BLE001
                log(f"failed to deserialize response for {_wire}: {exc}")
                return response

        self.create_service(srv_type, local, _cb, callback_group=group)
        log(f"relaying service {local} ({type_name}) calls to peer")

    # -- frame dispatch (called directly from the SocketLink reader thread) --

    def on_frame(self, kind, call_id, name, payload):
        if kind == FRAME_TOPIC:
            self._handle_topic_frame(name, payload)
        elif kind == FRAME_SVC_REQUEST:
            self._handle_service_request(call_id, name, payload)
        elif kind == FRAME_SVC_RESPONSE:
            self._handle_service_response(call_id, name, payload)
        else:
            log(f"WARNING: dropping frame with unknown kind {kind} for {name}")

    def _handle_topic_frame(self, topic, payload):
        entry = self._sinks.get(topic)
        if entry is None:
            log(f"WARNING: dropping frame for unconfigured topic {topic}")
            return
        pub, msg_class = entry
        try:
            msg = deserialize_message(payload, msg_class)
        except Exception as exc:  # noqa: BLE001
            log(f"failed to deserialize frame for {topic}: {exc}")
            return
        pub.publish(msg)

    def _handle_service_request(self, call_id, name, payload):
        entry = self._real_clients.get(name)
        if entry is None:
            log(f"WARNING: dropping service request for unconfigured service {name}")
            return
        client, srv_type = entry
        try:
            request = deserialize_message(payload, srv_type.Request)
        except Exception as exc:  # noqa: BLE001
            log(f"failed to deserialize request for {name}: {exc}")
            return
        if not client.service_is_ready():
            log(f"WARNING: real server for {name} is not available yet, dropping request")
            return

        future = client.call_async(request)

        def _done(fut, _call_id=call_id, _name=name):
            try:
                response = fut.result()
            except Exception as exc:  # noqa: BLE001
                log(f"real service call to {_name} failed: {exc}")
                return
            self.link.send(FRAME_SVC_RESPONSE, _call_id, _name, serialize_message(response))

        future.add_done_callback(_done)

    def _handle_service_response(self, call_id, name, payload):
        with self._svc_lock:
            pending = self._pending_calls.get(call_id)
        if pending is None:
            log(f"WARNING: dropping late/unknown service response for {name}")
            return
        event, holder = pending
        holder["payload"] = payload
        event.set()


def main():
    topic_entries = parse_entries(os.environ.get("BRIDGE_TOPICS", ""))
    service_entries = parse_entries(os.environ.get("BRIDGE_SERVICES", ""))
    if not topic_entries and not service_entries:
        log("WARNING: BRIDGE_TOPICS and BRIDGE_SERVICES are both empty, nothing to bridge")

    if SIDE == "dds":
        topic_source_dir, topic_sink_dir = "up", "down"
        service_real_client_dir = "down"
    elif SIDE == "zenoh":
        topic_source_dir, topic_sink_dir = "down", "up"
        service_real_client_dir = "up"
    else:
        raise SystemExit(f"BRIDGE_SIDE must be 'dds' or 'zenoh', got '{SIDE}'")

    topic_sources = [e for e in topic_entries if e[3] == topic_source_dir]
    topic_sinks = [e for e in topic_entries if e[3] == topic_sink_dir]
    service_real_clients = [e for e in service_entries if e[3] == service_real_client_dir]
    service_fake_servers = [e for e in service_entries if e[3] != service_real_client_dir]

    node_holder = {}

    def dispatch(kind, call_id, name, payload):
        node = node_holder.get("node")
        if node is not None:
            node.on_frame(kind, call_id, name, payload)

    rclpy.init()
    link = SocketLink(SOCKET_ROLE, SOCKET_HOST, SOCKET_PORT, dispatch)
    node = RelayNode(link, topic_sources, topic_sinks, service_real_clients, service_fake_servers)
    node_holder["node"] = node

    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
