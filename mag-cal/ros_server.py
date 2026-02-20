import threading
import asyncio
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
import websockets

# CONFIGURATION
# If your tool connects strictly to "ws://localhost/ws" (no port specified),
# it implies Port 80. Port 80 requires sudo/root on Linux.
# If you can configure the tool's port, change this to 8080 or 9090.
WS_PORT = 8080
WS_PATH = "/ws"
ROS_TOPIC = "/imu/mag"


class MagBridgeNode(Node):
    def __init__(self, loop):
        super().__init__("mag_ws_bridge")
        self.loop = loop
        self.ws_client = None
        self.is_streaming = False

        # Subscribe to the magnetometer data
        self.subscription = self.create_subscription(
            MagneticField, ROS_TOPIC, self.mag_callback, 10
        )
        self.get_logger().info(f"Subscribed to {ROS_TOPIC}")

    def mag_callback(self, msg):
        # Only process if we have a connected client who asked to start (0x11)
        if self.ws_client and self.is_streaming:
            x = float(msg.magnetic_field.x * 1e6)
            y = float(msg.magnetic_field.y * 1e6)
            z = float(msg.magnetic_field.z * 1e6)

            # Protocol: 0x01 <F32> <F32> <F32> (Little Endian)
            # < = Little Endian
            # B = Unsigned Char (1 byte)
            # f = Float (4 bytes)
            try:
                payload = struct.pack("<Bfff", 0x01, x, y, z)

                # Send data safely into the asyncio loop from the ROS thread
                asyncio.run_coroutine_threadsafe(
                    self.ws_client.send(payload), self.loop
                )
            except Exception as e:
                self.get_logger().error(f"Error packing/sending data: {e}")

    def set_client(self, ws):
        self.ws_client = ws
        self.get_logger().info("Client connected via WebSocket")

    def remove_client(self):
        self.ws_client = None
        self.is_streaming = False
        self.get_logger().info("Client disconnected")

    def start_streaming(self):
        self.is_streaming = True
        self.get_logger().info("Received 0x11: STARTED streaming mag data")

    def stop_streaming(self):
        self.is_streaming = False
        self.get_logger().info("Received 0x10: STOPPED streaming mag data")


async def ws_handler(websocket, node):
    node.set_client(websocket)

    try:
        async for message in websocket:
            # We expect bytes. If the tool sends text, we might need message.encode()
            # But usually raw sockets send bytes.
            if isinstance(message, str):
                data = message.encode("utf-8")  # Fallback if text
            else:
                data = message

            if len(data) > 0:
                cmd = data[0]

                if cmd == 0x11:
                    node.start_streaming()
                elif cmd == 0x10:
                    node.stop_streaming()
                else:
                    print(f"Unknown command byte: {hex(cmd)}")

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        node.remove_client()


def ros_spin_thread(node):
    rclpy.spin(node)


async def main():
    rclpy.init()

    # Get the current asyncio loop (to pass to ROS node)
    loop = asyncio.get_running_loop()

    node = MagBridgeNode(loop)

    # Run ROS in a background thread so it doesn't block the WebSocket loop
    t = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    t.start()

    print(f"Starting WebSocket server on ws://0.0.0.0:{WS_PORT}{WS_PATH}")
    print(f"Listening for ROS topic: {ROS_TOPIC}...")

    # Start the WebSocket Server
    async with websockets.serve(lambda ws: ws_handler(ws, node), "0.0.0.0", WS_PORT):
        # Keep the main thread alive for the async server
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
