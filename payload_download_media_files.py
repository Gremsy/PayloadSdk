import requests
import re
import sys
from typing import List

href_elements: List[str] = []
listed = False
download_directory = ""
udp_ip_target = "192.168.55.1"

# encode spaces in URL
def encode_url_spaces(name: str) -> str:
    return name.replace(" ", "%20")

# decode spaces in URL
def decode_url_spaces(name: str) -> str:
    return name.replace("%20", " ")

# Ensure the path ends with a trailing slash
def ensure_trailing_slash(path: str) -> str:
    return path if path.endswith('/') else path + '/'

# Check if the file name has an image extension
def is_image_extension(file_name: str) -> bool:
    image_extensions = [".jpg", ".jpeg", ".png", ".bmp", ".gif"]
    return any(file_name.lower().endswith(ext) for ext in image_extensions)

# Check if the file name has a video extension
def is_video_extension(file_name: str) -> bool:
    video_extensions = [".mp4", ".avi", ".mov", ".mkv", ".wmv"]
    return any(file_name.lower().endswith(ext) for ext in video_extensions)

# Check if the path is a directory path (starts with /)
def is_directory_path(path: str) -> bool:
    return path.startswith('/')

# Check if the string is a valid IP address
def is_ip_address(ip: str) -> bool:
    pattern = r"^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$"
    return bool(re.match(pattern, ip))

# Fetch and list media files from the given URL
def directory_listing(url: str) -> None:
    global href_elements, listed
    href_elements.clear()
    listed = True

    try:
        response = requests.get(url)
        response.raise_for_status()
        content = response.text

        pattern = r'<a href="/delete/(.*?)" class="delete-link"'
        matches = re.finditer(pattern, content)

        for match in matches:
            image_url = match.group(1)
            image_name = image_url.split('/')[-1]
            href_elements.append(encode_url_spaces(image_name))

    except requests.RequestException as e:
        print(f"Request failed: {e}")

# Download a file from the specified URL to the local directory
def download_file(url: str, file_name: str) -> None:
    download_url = f"{url}/download/{file_name}"
    local_file_name = decode_url_spaces(file_name)

    if download_directory:
        full_path = ensure_trailing_slash(download_directory) + local_file_name
    else:
        full_path = local_file_name

    print(f"Starting download from: {download_url}")
    print(f"Saving to: {full_path}")

    try:
        response = requests.get(download_url, stream=True)
        response.raise_for_status()

        total_size = int(response.headers.get('content-length', 0))
        downloaded_size = 0

        with open(full_path, 'wb') as f:
            for chunk in response.iter_content(chunk_size=8192):
                if chunk:
                    f.write(chunk)
                    downloaded_size += len(chunk)
                    print(f"\rDownloading... {downloaded_size}/{total_size} bytes", end='')

        print(f"\nDownload completed: {local_file_name}")

    except Exception as e:
        print(f"Download failed: {e}")

# Main function to handle user interaction and media file downloads
def main():
    global download_directory

    if len(sys.argv) == 1:
        print("The download directory is in current folder.")
    else:
        if is_directory_path(sys.argv[1]):
            download_directory = sys.argv[1]
            print(f"The download directory: {download_directory}")
        else:
            print("The download directory is in current folder.")

    print(f"IP Address: {udp_ip_target}")
    base_url = f"http://{udp_ip_target}:8000"

    while True:
        print("\n----")
        print("Select an option:")
        print("  1. List media files")
        print("  2. Download a Image or a Video")
        print("  3. Download all Images")
        print("  4. Download all Videos")
        print("  Enter 'q' to quit")
        choice = input("Choice: ").strip().lower()
        print("")

        if choice == "1":
            print("Listing items...")
            directory_listing(f"{base_url}/list-file")
            print("")
            for element in href_elements:
                print(decode_url_spaces(element))

        elif choice == "2":
            directory_listing(f"{base_url}/list-file")
            print("")
            for element in href_elements:
                print(decode_url_spaces(element))
            print("--")
            name_input = input("Downloading a image or video. Enter the name: ").strip()
            name_parts = name_input.split()

            if len(name_parts) != 1:
                print("Invalid choice. Please try again.")
            else:
                name = name_parts[0]
                download_file(base_url, encode_url_spaces(name))

        elif choice == "3":
            print("Downloading all Images...")
            if not listed:
                directory_listing(f"{base_url}/list-file")
                print("")
            for element in href_elements:
                if is_image_extension(decode_url_spaces(element)):
                    download_file(base_url, element)

        elif choice == "4":
            print("Downloading all Videos...")
            if not listed:
                directory_listing(f"{base_url}/list-file")
                print("")
            for element in href_elements:
                if is_video_extension(decode_url_spaces(element)):
                    download_file(base_url, element)

        elif choice == "q":
            break

        else:
            print("Invalid choice. Please try again.")

if __name__ == "__main__":
    main()