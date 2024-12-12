import zmq
from motion.message import detection_pb2  # Import the generated code from the .proto file
import time

class VisionSocket:
    def __init__(self, vision_node_ip) -> None:
        self._vision_node_ip = vision_node_ip

        # Set up the context and socket
        self.context = zmq.Context()
        self._socket = self.context.socket(zmq.REQ)  # Create a request socket

        self._socket.connect(self._vision_node_ip)  # Connect to the server

    def get_taskboard_pose_in_world(self):
        # Send a message
        message = "pose"
        self._socket.send_string(message)
        print(f"Sent: {message}")

        # Wait for a reply
        reply = self._socket.recv()
        print("Received raw bytes:", reply)

        # Decode the received data using Protocol Buffers
        vision_message = detection_pb2.Vision()
        vision_message.ParseFromString(reply)  # Parse the incoming bytes

        return vision_message.pose
    
    def get_slider_moving_pixels(self):
        # Send a message
        message = "direction"
        self._socket.send_string(message)
        print(f"Sent: {message}")

        # Wait to prevent heavy blurring
        time.sleep(1.0)

        # Parse the reply and get pixel distance to move
        reply = self.socket.recv()
        vision_message = detection_pb2.Vision()
        vision_message.ParseFromString(reply)  # Parse the incoming bytes
        pixel_distance = vision_message.direction
        
        return pixel_distance