import zmq
import detection_pb2  # Import the generated code from the .proto file

# Set up the context and socket
context = zmq.Context()
socket = context.socket(zmq.REQ)  # Create a request socket

socket.connect("tcp://192.168.1.13:5556")  # Connect to the server

# Send a message
message = "pose"
socket.send_string(message)
print(f"Sent: {message}")

# Wait for a reply
reply = socket.recv()
print("Received raw bytes:", reply)

# Decode the received data using Protocol Buffers
vision_message = detection_pb2.Vision()
vision_message.ParseFromString(reply)  # Parse the incoming bytes

# Access the decoded data
print(f"Success: {vision_message.success}")
print(f"Pose: {vision_message.pose}")
print(f"Direction: {vision_message.direction}")

# Clean up
socket.close()
context.term()
