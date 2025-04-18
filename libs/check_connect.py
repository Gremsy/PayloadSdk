import argparse
import signal
import sys
import time
from libs.payload_sdk import PayloadInterface

my_payload = None

def quit_handler(sig, frame):
    global my_payload
    my_payload.sdk_quit()
    sys.exit(0)

def main():
    # Thiết lập tham số dòng lệnh
    parser = argparse.ArgumentParser(description="Kiểm tra kết nối với Payload")
    parser.add_argument('--type', choices=['udp', 'serial'], default='udp', help="Loại kết nối: 'udp' hoặc 'serial'")
    parser.add_argument('--ip', help="Địa chỉ IP UDP (ví dụ: 192.168.55.1)")
    parser.add_argument('--port', type=int, help="Cổng UDP (ví dụ: 14566)")
    parser.add_argument('--serial-port', help="Cổng serial (ví dụ: /dev/ttyUSB0)")
    parser.add_argument('--baudrate', type=int, help="Tốc độ baud serial (ví dụ: 115200)")
    parser.add_argument('--sys-id', type=int, default=1, help="ID hệ thống")
    parser.add_argument('--comp-id', type=int, default=200, help="ID thành phần")
    args = parser.parse_args()

    # Khởi tạo đối tượng PayloadInterface
    my_payload = PayloadInterface(
        connection_type=args.type,
        ip=args.ip,
        port=args.port,
        serial_port=args.serial_port,
        baudrate=args.baudrate,
        sys_id=args.sys_id,
        comp_id=args.comp_id
    )

    # Đăng ký xử lý ngắt Ctrl+C
    signal.signal(signal.SIGINT, quit_handler)

    # Khởi tạo kết nối
    if not my_payload.sdk_init_connection():
        print("Không thể khởi tạo kết nối")
        sys.exit(1)

    print("Đang đợi tín hiệu từ payload...")
    # Kiểm tra kết nối với payload
    if my_payload.check_payload_connection():
        print("Đã kết nối với payload!")
    else:
        print("Không thể kết nối với payload")
        my_payload.sdk_quit()
        sys.exit(1)

    # Chạy vòng lặp vô hạn để giữ chương trình hoạt động
    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()