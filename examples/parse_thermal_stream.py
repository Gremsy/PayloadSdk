#!/usr/bin/env python3
import sys
import os

# Add the libs directory to the path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'libs'))

# Import config first to setup environment automatically
from config import config

import socket
import threading
import queue
import cv2
import numpy as np
import zlib
import struct
import sys
import time

# ==================== CONFIGURATION ====================
PAYLOAD_IP = config.connection.UDP_IP_TARGET
PAYLOAD_PORT = 8080
BUFFER_SIZE = 65536

WIDTH = 640
HEIGHT = 512

# ==================== GLOBAL VARIABLES ====================
data_queue = queue.Queue(maxsize=30)  # Increased to 30 to buffer more frames, reduce dropping

class TempPoint:
    def __init__(self):
        self.x = -1
        self.y = -1
        self.raw_data = -1
        self.temp = 0.0

coldest_point = TempPoint()
hottest_point = TempPoint()
cursor_point = TempPoint()
cursor_point.x = WIDTH // 2
cursor_point.y = HEIGHT // 2

# Grayscale color palette (can be replaced with thermal palette later) - Converted to NumPy array for speed
COLOR_PALETTE_RGBA = np.array([[i, i, i] for i in range(256)], dtype=np.uint8)

# ==================== UTILITY FUNCTIONS ====================

def map_value(x, in_min, in_max, out_min, out_max):
    if in_max == in_min:
        return out_min
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def uncompress_data(compressed_data):
    try:
        decompressed_size = WIDTH * HEIGHT * 2
        raw_decompressed = zlib.decompress(compressed_data)
        if len(raw_decompressed) != decompressed_size:
            raise ValueError(f"Decompressed size mismatch: {len(raw_decompressed)}")
        delta_frame = np.frombuffer(raw_decompressed, dtype=np.int16)  # Use NumPy for speed
        decoded = np.cumsum(delta_frame)  # Vectorized delta decode (cumsum instead of loop)
        decoded = decoded.astype(np.uint16)
        return decoded.tolist()  # Convert back to list if needed, but can keep NumPy
    except Exception as e:
        print(f"[DECOMPRESSION ERROR] {e}")
        return None

def get_temp_image(data):
    global coldest_point, hottest_point, cursor_point
    # Convert data to 2D NumPy array for vectorization
    data_np = np.array(data).reshape(HEIGHT, WIDTH).astype(np.float32)
    temps = data_np / 100.0 - 273.15
    
    # Find min/max vectorized
    coldest_idx = np.argmin(temps)
    hottest_idx = np.argmax(temps)
    coldest_y, coldest_x = np.unravel_index(coldest_idx, temps.shape)
    hottest_y, hottest_x = np.unravel_index(hottest_idx, temps.shape)
    
    coldest_point.temp = temps[coldest_y, coldest_x]
    coldest_point.x = coldest_x
    coldest_point.y = coldest_y
    coldest_point.raw_data = data[coldest_idx]
    
    hottest_point.temp = temps[hottest_y, hottest_x]
    hottest_point.x = hottest_x
    hottest_point.y = hottest_y
    hottest_point.raw_data = data[hottest_idx]
    
    # Cursor temp
    cursor_point.temp = temps[cursor_point.y, cursor_point.x]

def apply_palette(data, color_palette):
    # Convert data to 2D NumPy array
    data_np = np.array(data).reshape(HEIGHT, WIDTH).astype(np.float32)
    min_raw = coldest_point.raw_data
    max_raw = hottest_point.raw_data
    
    # Vectorized mapping and clip
    if max_raw == min_raw:
        gray_vals = np.zeros_like(data_np, dtype=np.int32)
    else:
        gray_vals = np.clip(((data_np - min_raw) / (max_raw - min_raw) * 255).astype(np.int32), 0, 255)
    
    # Apply palette vectorized
    img_flat = color_palette[gray_vals.flatten()]
    img = img_flat.reshape(HEIGHT, WIDTH, 3)
    
    return img

def draw_temp_point(image, temp_point, color):
    h, w = image.shape[:2]
    x, y = temp_point.x, temp_point.y
    if x < 0 or y < 0 or x >= w or y >= h:
        return
    
    # Draw crosshair
    thickness = 1
    cv2.line(image, (x + 10, y), (x + 5, y), color, thickness)
    cv2.line(image, (x - 10, y), (x - 5, y), color, thickness)
    cv2.line(image, (x, y + 10), (x, y + 5), color, thickness)
    cv2.line(image, (x, y - 10), (x, y - 5), color, thickness)
    
    # Create text
    text = f"{temp_point.temp:.2f}C"
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.4
    thickness = 1
    (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    
    # Calculate offset so text does not overlap cursor
    margin = 5  # extra spacing between text and cursor
    
    if x <= w // 2:  # point on the left side of the image → text shifts right
        offset_x = margin
    else:            # point on the right side of the image → text shifts left
        offset_x = -(text_width + margin)

    if y < h // 2:   # point in the top half → text below the point
        offset_y = text_height + margin
    else:            # point in the bottom half → text above the point
        offset_y = -(margin)

    text_pos = (x + offset_x, y + offset_y)
    
    # Draw text
    cv2.putText(image, text, text_pos, font, font_scale, color, thickness, cv2.LINE_AA)

def push_raw_data_to_queue(data):
    try:
        data_queue.put(data, timeout=0.1)  # Reduced timeout for faster enqueue
    except queue.Full:
        print("[WARNING] Queue full, skipping data")  # Still log, but less frequent due to larger queue

# ==================== IMAGE PROCESSING THREAD ====================

def raw_data_process_thread():
    cv2.namedWindow("Thermal Image", cv2.WINDOW_AUTOSIZE)
    # frame_count = 0
    # start_time = time.time()
    while True:
        try:
            data = data_queue.get(timeout=1.0)
            if data is None:
                break
            get_temp_image(data)
            img_rgb = apply_palette(data, COLOR_PALETTE_RGBA)
            draw_temp_point(img_rgb, hottest_point, (0, 0, 255))  # Red for hottest
            draw_temp_point(img_rgb, coldest_point, (255, 0, 0))  # Blue for coldest
            draw_temp_point(img_rgb, cursor_point, (0, 255, 0))  # Green for cursor
            cv2.imshow("Thermal Image", img_rgb)
            
            # frame_count += 1
            # # Calculate FPS every second
            # elapsed = time.time() - start_time
            # if elapsed >= 1.0:
            #     fps = frame_count / elapsed
            #     # print(f"[FPS] {fps:.2f} FPS")
            #     frame_count = 0
            #     start_time = time.time()
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        except queue.Empty:
            continue
        except Exception as e:
            print(f"[DISPLAY ERROR] {e}")
            continue
    cv2.destroyAllWindows()

# ==================== MAIN FUNCTION ====================

def main():
    # Start image processing thread
    frame_thread = threading.Thread(target=raw_data_process_thread, daemon=True)
    frame_thread.start()

    # Create socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)  # Timeout to avoid hang if connection fails

    try:
        sock.connect((PAYLOAD_IP, PAYLOAD_PORT))
        print(f"[+] Successfully connected to {PAYLOAD_IP}:{PAYLOAD_PORT}")
    except Exception as e:
        print(f"[CONNECTION ERROR] {e}")
        return

    try:
        while True:
            # Receive data size (8 bytes, uint64 little-endian - matches size_t in C++)
            size_data = b""
            while len(size_data) < 8:
                chunk = sock.recv(8 - len(size_data))
                if not chunk:
                    print("[ERROR] Lost connection to server while receiving data size")
                    break
                size_data += chunk
            if len(size_data) != 8:
                print("[ERROR] Did not receive full 8 bytes for data size")
                break

            data_size = struct.unpack('<Q', size_data)[0]
            # print(f"[DEBUG] Received data size: {data_size} bytes")  # Log for verification

            if data_size <= 0 or data_size > 2 * 1024 * 1024:  # Limit to max 2MB/frame
                print(f"[WARNING] Invalid data size: {data_size}")
                continue

            # Receive raw data
            socket_data = b""
            while len(socket_data) < data_size:
                chunk = sock.recv(min(data_size - len(socket_data), BUFFER_SIZE))
                if not chunk:
                    print("[ERROR] Incomplete data received")
                    break
                socket_data += chunk
            if len(socket_data) != data_size:
                print(f"[ERROR] Received {len(socket_data)} bytes, expected {data_size} bytes")
                continue

            # Decompress data
            raw_data = uncompress_data(socket_data)
            if raw_data is None:
                continue

            # Push data into queue
            push_raw_data_to_queue(raw_data)

    except KeyboardInterrupt:
        print("[INFO] Ctrl+C detected, exiting program...")
    except socket.timeout:
        print("[WARNING] Timeout while receiving data")
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        # Close socket
        sock.close()
        print("[INFO] Socket connection closed")
        # Push None into queue to signal processing thread to exit
        push_raw_data_to_queue(None)
        frame_thread.join()

if __name__ == "__main__":
    main()