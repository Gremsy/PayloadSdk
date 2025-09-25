'''
 * This sample will show the way to extract the tiff files to get full-frame temperatures
'''

import sys
import signal
import cv2
import numpy as np

# Enable this flag to test palette color application
APPLY_PALETTE_COLOR = True

# Palette color RGBA (simplified grayscale palette)
if APPLY_PALETTE_COLOR:
    colorpalette_rgba = [[i, i, i, 0] for i in range(256)]
    array_raw = np.zeros((512 * 640), dtype=np.uint16)
    raw_max = 0
    raw_min = 65536

# Signal handler for Ctrl+C
def signal_handler(sig, frame):
    print("\nTERMINATING AT USER REQUEST")
    cv2.destroyAllWindows()  # Close all OpenCV windows
    sys.exit(0)

# Register signal handler
signal.signal(signal.SIGINT, signal_handler)

def get_raw_temp_data(filepath, w, h):
    global raw_max, raw_min, array_raw
    # Read image in unchanged format (16-bit grayscale expected)
    image = cv2.imread(filepath, cv2.IMREAD_UNCHANGED)
    if image is None:
        print("Could not open or find the image")
        return None, -1

    # Check if image is 16-bit grayscale
    if image.dtype != np.uint16:
        print("Image is not 16-bit grayscale.")
        return None, -1

    if w != image.shape[1] or h != image.shape[0]:
        print("Input resolution error.")
        return None, -1

    # Initialize temperature data array
    temp_data = np.zeros((w * h), dtype=np.float32)

    for y in range(h):
        for x in range(w):
            # Convert raw to temperature (Celsius)
            temp_data[y * w + x] = image[y, x] / 100.0 - 273.15

            if APPLY_PALETTE_COLOR:
                array_raw[y * w + x] = image[y, x]
                raw_max = max(raw_max, array_raw[y * w + x])
                raw_min = min(raw_min, array_raw[y * w + x])

    return temp_data, 0

def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def apply_palette(data, result, color_palette):
    for y in range(512):
        for x in range(640):
            gray_value = int(map_value(data[y * 640 + x], raw_min, raw_max, 0, 255))
            result[y, x] = [
                color_palette[gray_value][0],  # R
                color_palette[gray_value][1],  # G
                color_palette[gray_value][2],  # B
            ]

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 payload_extract_tiff_file.py <path_to_image>")
        return -1

    width = 640
    height = 512

    # Get temperature data from TIFF file
    temp_data, ret = get_raw_temp_data(sys.argv[1], width, height)
    if ret < 0:
        print("Get Raw Temp Data error")
        return -1

    print("Temp Data:")
    for i in range(height):
        for j in range(width):
            print(f"{temp_data[i * width + j]:.2f} ", end="")
        print()

    if APPLY_PALETTE_COLOR:
        # Create RGB image
        image_rgb = np.zeros((512, 640, 3), dtype=np.uint8)
        apply_palette(array_raw, image_rgb, colorpalette_rgba)

        # Convert RGB to BGR to show (giống C++)
        image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)

        # Display image
        cv2.imshow("Image RGB", image_bgr)
        while True:
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or cv2.getWindowProperty("Image RGB", cv2.WND_PROP_VISIBLE) < 1:
                cv2.destroyAllWindows()
                break
            
    return 0

if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        # Ensure clean exit on Ctrl+C
        cv2.destroyAllWindows()
        print("\nTERMINATING AT USER REQUEST")
        sys.exit(0)