
import os
import torch
import cv2
import socket
import numpy as np
import time
from ultralytics import YOLO
from multiprocessing import Process, Queue
import struct

#  Load YOLOv8 Model with CUDA
MODEL_PATH = r"best.pt"
device = torch.device("cuda" if torch.cuda.is_available()
                      else "cpu")  # Use CUDA if available
print(f"Using device: {device}")
model = YOLO(MODEL_PATH).to(device).eval()  # Load model on CUDA

#  Server Configuration
DEFAULT_NUM_UAVS = 3  # Change this based on how many UAVs are spawned
UNREAL_ENGINE_PORT = 5000  # Starting port for UAVs

UNREAL_ENGINE_IP = "127.0.0.1"
UNREAL_ENGINE_PORT = 5000
# UDP Port for receiving UAV count
PYTHON_UDP_PORT = 6000

# Renter target orignal values
UE_RENDER_WIDTH = 1707
UE_RENDER_HEIGHT = 1067

unreal_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP socket

#  Function to calculate the center of a bounding box


def recive_uav_count():
    """ Listens for the number of UAVs from UE """
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind(("127.0.0.1", PYTHON_UDP_PORT))
    udp_socket.settimeout(5)  # Timeout after 5 seconds

    print(
        f"[INFO] Waiting for UAV count from Unreal Engine on Port {PYTHON_UDP_PORT}...")

    try:
        data, addr = udp_socket.recvfrom(4)  # Receive 4 bytes (int32)
        DEFAULT_NUM_UAVS = struct.unpack("i", data)[0]
        print(f"[INFO] Received UAV count: {DEFAULT_NUM_UAVS} from {addr}")
    except socket.timeout:
        print("[ERROR] Timeout! No UAV count received from Unreal Engine.")
        DEFAULT_NUM_UAVS = 3  # Set a fallback value
    finally:
        udp_socket.close()

    return DEFAULT_NUM_UAVS


def get_bbox_center(x1, y1, x2, y2):
    """ Returns the center coordinates of a bounding box. """
    center_x = (x1 + x2) / 2
    center_y = (y1 + y2) / 2

    # Due the original resolution is different we need to scale the output
    # in this case resolutions is  (1707 x 1067)

    """
    Follow the fomula:

    Scaled Center X = (Original width / 640) Yolo Center X (bounding box)
    Scaled Center Y = (Original Height / 640) Yolo Center Y (bounding box)


    """

    scaled_x = (UE_RENDER_WIDTH / 640) * center_x
    scaled_y = (UE_RENDER_HEIGHT / 640) * center_y

    return int(scaled_x), int(scaled_y)


#  Function to start UAV stream capture
def capture_stream(uav_id, port, queue):
    """ Captures video frames from a UAV and stores them in a queue. """
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(("127.0.0.1", port))
    server_socket.listen(1)
    print(f"[INFO] UAV {uav_id} waiting on port {port}...")

    client_socket, addr = server_socket.accept()
    print(f"[INFO] UAV {uav_id} connected from {addr}")

    while True:
        try:
            data = b""
            while True:
                packet = client_socket.recv(65536)
                if not packet:
                    print(f"[ERROR] UAV {uav_id} connection lost.")
                    break
                data += packet
                if b"\xFF\xD9" in packet:  # JPEG end marker
                    break

            if not data:
                break

            #  Decode the frame
            frame_array = np.frombuffer(data, dtype=np.uint8)
            frame = cv2.imdecode(frame_array, cv2.IMREAD_COLOR)

            if frame is not None and not queue.full():
                queue.put(frame)

        except Exception as e:
            print(f"[ERROR] UAV {uav_id} stream error: {e}")
            break

    client_socket.close()
    server_socket.close()


#  Function to process frames & detect persons
def process_uav(uav_id, queue):
    """ Processes video frames from the UAV queue and detects persons using YOLO. """
    while True:
        if not queue.empty():
            frame = queue.get()

            #  Convert frame to CUDA tensor
            img_tensor = (
                torch.from_numpy(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                .permute(2, 0, 1)  # Convert HWC -> CHW
                .float()
                .div(255.0)  # Normalize
                .to(device)  # Move to CUDA
                .unsqueeze(0)  # Add batch dimension
            )

            #  YOLOv8 Inference with CUDA
            start_time = time.time()
            with torch.inference_mode():
                results = model(img_tensor)  # Model inference
            end_time = time.time()
            print(
                f"[INFO] UAV {uav_id} Inference Time: {end_time - start_time:.4f} sec")

            #  Process Detections & Draw Bounding Boxes
            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu(
                    ).detach().float().tolist()
                    conf = float(box.conf[0].cpu().detach())
                    cls = int(box.cls[0].cpu().detach())

                    if model.names[cls] == "person" and conf > 0.90:  # Only detect people
                        label = f"Person {conf:.2f}"
                        cv2.rectangle(frame, (int(x1), int(y1)), (int(
                            x2), int(y2)), (0, 255, 0), 2)  # Green Box
                        cv2.putText(frame, label, (int(x1), int(y1) - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                        #  Calculate the center of the bounding box
                        center_x, center_y = get_bbox_center(x1, y1, x2, y2)
                        print(
                            f"[INFO] UAV {uav_id}: Person detected at center -> X: {center_x}, Y: {center_y}")
                        # send data to unreal engine
                        message = struct.pack(
                            "iii", uav_id, center_x, center_y)
                        unreal_socket.sendto(
                            message, (UNREAL_ENGINE_IP, UNREAL_ENGINE_PORT))
                        try:
                            sent_bytes = unreal_socket.sendto(
                                message, ("127.0.0.1", 7000))
                            if sent_bytes > 0:
                                print(
                                    f"[INFO] Sent {sent_bytes} bytes to Unreal Engine.")
                            else:
                                print(
                                    "[ERROR] Failed to send data to Unreal Engine.")
                        except Exception as e:
                            print(f"[ERROR] UDP send error: {e}")
            #  Show each UAV's video feed with detections
            cv2.imshow(f"UAV {uav_id} Detection", frame)
            cv2.waitKey(1)

        if cv2.waitKey(1) & 0xFF == ord("q"):  # Stop on 'q' key
            break


#  Main function to start everything
def start_uav_detection():
    """ Initializes and starts the UAV detection system with given UAV count. """
    # num_uavs = recive_uav_count()  # Get the number of UAVs from Unreal
    num_uavs = 3
    print(f"[INFO] Starting detection for {num_uavs} UAVs...")

    uav_ports = [UNREAL_ENGINE_PORT + i for i in range(num_uavs)]
    uav_queues = [Queue(maxsize=10) for _ in range(num_uavs)]

    #  Start UAV Capture Processes
    processes = []
    for i in range(num_uavs):
        p = Process(target=capture_stream, args=(
            i + 1, uav_ports[i], uav_queues[i]))
        p.start()
        processes.append(p)

    #  Start UAV Processing Threads
    for i in range(num_uavs):
        Process(target=process_uav, args=(i + 1, uav_queues[i])).start()

    #  Keep script running until user exits
    print("[INFO] Press 'q' to stop all UAVs...")
    while True:
        if cv2.waitKey(1) & 0xFF == ord("q"):
            print("[INFO] Stopping all UAVs...")
            break

    #  Cleanup
    for p in processes:
        p.terminate()
    cv2.destroyAllWindows()


#  Run the system only if executed directly
if __name__ == "__main__":
    # Change this to the desired number of UAVs
    start_uav_detection()

# ===================================================================
