#!/usr/bin/env python3
import sys
import os

# Add the libs directory to the path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'libs'))

# Import config first to setup environment automatically
from config import config

import requests
import re
import signal
import getpass
from typing import List, Optional, Tuple

href_elements: List[str] = []
listed = False
download_directory = ""
time_to_exit = False

# Some payloads ask for a login before serving media files. Probe the server at
# run time instead of relying on the payload model. Credentials are typed in by
# the user and never written to disk.
LOGIN_MAX_ATTEMPTS = 3

# One session for the whole run: it keeps the cookie the server hands out at
# login and replays it on the listing, download and delete requests.
session = requests.Session()

# Base URL "http://<ip>:8000", needed to sign in again when a session expires.
base_url = ""

# Get IP from config instead of hardcoding
udp_ip_target = config.connection.UDP_IP_TARGET

# Signal handler for quitting
def quit_handler(sig, frame):
    global time_to_exit
    print("\n\nTERMINATING AT USER REQUEST")
    print("Exiting download media files program...")
    time_to_exit = True
    sys.exit(0)

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

# An unauthenticated page comes back with HTTP 200, so the status code alone
# cannot tell a valid session from an expired one.
def is_login_page(html: str) -> bool:
    return "/smb-login" in html or "smb-username" in html

# Post the credentials. On success the server sets a session cookie that the
# shared requests.Session replays on the following requests.
def do_login(url: str, user: str, password: str) -> Tuple[bool, str]:
    try:
        response = session.post(f"{url}/smb-login",
                                json={"username": user, "password": password},
                                timeout=15)
    except requests.RequestException as e:
        return False, str(e)

    try:
        body = response.json()
    except ValueError:
        return False, f"unexpected reply from server (HTTP {response.status_code})"

    if body.get("ok"):
        return True, ""
    return False, body.get("error", f"HTTP {response.status_code}")

# Ask for the credentials and retry a few times on rejection.
def do_interactive_login(url: str) -> bool:
    print("This payload requires a login to access media files.")

    for attempt in range(1, LOGIN_MAX_ATTEMPTS + 1):
        try:
            user = input("Username: ").strip()
            password = getpass.getpass("Password: ")
        except EOFError:
            return False

        if not user or not password:
            print("Username and password must not be empty.")
            continue

        ok, error = do_login(url, user, password)
        if ok:
            print("Login successful.")
            return True

        print(f"Login failed: {error} (attempt {attempt}/{LOGIN_MAX_ATTEMPTS})")

    return False

# Sign in only when the server asks for it. An unauthenticated page answers 200
# with the login form; a payload still detecting its cameras answers 404.
def ensure_authenticated(url: str) -> bool:
    try:
        response = session.get(f"{url}/list-file", timeout=15, allow_redirects=False)
    except requests.RequestException as e:
        print(f"Cannot reach the media server: {e}")
        return False

    if response.status_code == 404:
        print("Media files are not available on this payload: the payload type has not "
              "been detected yet. Wait for the payload to finish starting up and try again.")
        return False

    if response.status_code != 200:
        print(f"Media server replied HTTP {response.status_code}")
        return False

    if not is_login_page(response.text):
        return True     # this payload serves the media files without a login

    return do_interactive_login(url)

# Read the storage choices straight out of the media page, so the SDK does not
# need to know which payload models have which storage.
def parse_storage_options(html: str) -> Tuple[List[Tuple[str, str]], str]:
    start = html.find('id="storage-select"')
    if start == -1:
        return [], ""       # this payload has a single storage

    end = html.find("</select>", start)
    if end == -1:
        return [], ""

    block = html[start:end]
    options: List[Tuple[str, str]] = []
    current = ""

    for match in re.finditer(r'<option value="([^"]*)"([^>]*)>([^<]*)</option>', block):
        value, attrs, label = match.group(1), match.group(2), match.group(3)
        options.append((value, label.strip()))
        if "selected" in attrs:
            current = value

    return options, current

# Tell the server which storage the following requests apply to.
def set_storage_source(url: str, option: str, allow_retry: bool = True) -> bool:
    global listed

    try:
        response = session.post(f"{url}/process-option",
                                json={"option": option}, timeout=15)
    except requests.RequestException as e:
        print(f"Request failed: {e}")
        return False

    if is_login_page(response.text):
        print("The session has expired.")
        if not allow_retry or not do_interactive_login(base_url):
            return False
        return set_storage_source(url, option, False)

    try:
        body = response.json()
    except ValueError:
        print(f"Could not switch storage: HTTP {response.status_code}")
        return False

    if body.get("ok"):
        listed = False      # the cached listing belongs to the old storage
        print(f"Storage source: {body.get('media_source', option)} "
              f"({body.get('directory', '')})")
        return True

    print(f"Could not switch storage: {body.get('error', response.status_code)}")
    return False

# Ask which storage to work on. Does nothing when the payload has only one.
def choose_storage_source(url: str) -> None:
    try:
        response = session.get(f"{url}/list-file", timeout=15, allow_redirects=False)
    except requests.RequestException:
        return

    options, current = parse_storage_options(response.text)
    if len(options) < 2:
        return

    print("")
    print("This payload has more than one storage. Select the one to work on:")
    for index, (value, label) in enumerate(options, start=1):
        suffix = "  (current)" if value == current else ""
        print(f"  {index}. {label}{suffix}")

    try:
        answer = input("Choice (Enter to keep the current one): ").strip()
    except EOFError:
        return

    if not answer:
        return

    if not answer.isdigit() or not 1 <= int(answer) <= len(options):
        print("Invalid choice, keeping the current storage.")
        return

    picked = options[int(answer) - 1][0]
    if picked != current:
        set_storage_source(url, picked)

# Fetch and list media files from the given URL
def directory_listing(url: str, allow_retry: bool = True) -> bool:
    global href_elements, listed

    href_elements.clear()
    listed = False

    try:
        response = session.get(url, timeout=15, allow_redirects=False)
        response.raise_for_status()
    except requests.RequestException as e:
        print(f"Request failed: {e}")
        return False

    content = response.text

    # An expired session returns the login page with a 200 status, which would
    # otherwise look like an empty storage. Sign in again and retry once.
    if is_login_page(content):
        print("The session has expired.")
        if not allow_retry or not do_interactive_login(base_url):
            return False
        return directory_listing(url, False)

    for match in re.finditer(r'<a href="/delete/(.*?)" class="delete-link"', content):
        image_name = match.group(1).split('/')[-1]
        href_elements.append(encode_url_spaces(image_name))

    listed = True
    return True

# Download a file from the specified URL to the local directory
def download_file(url: str, file_name: str, allow_retry: bool = True) -> bool:
    global time_to_exit

    download_url = f"{url}/download/{file_name}"
    local_file_name = decode_url_spaces(file_name)

    if download_directory:
        full_path = ensure_trailing_slash(download_directory) + local_file_name
    else:
        full_path = local_file_name

    print(f"Starting download from: {download_url}")
    print(f"Saving to: {full_path}")

    try:
        response = session.get(download_url, stream=True, timeout=30,
                               allow_redirects=False)

        # A redirect means the session is gone - report it instead of saving the
        # login page under the media file's name.
        if response.is_redirect:
            print("Download rejected: session expired, please log in again.")
            return False

        if response.status_code != 200:
            print(f"Download failed: HTTP {response.status_code}")
            return False

        # A web page answered with 200 can only be the login form. Without this
        # check it would be saved under the media file's name and look like a
        # corrupt photo. Sign in again and retry once.
        if "text/html" in response.headers.get("content-type", ""):
            print("The session has expired.")
            if not allow_retry or not do_interactive_login(base_url):
                return False
            return download_file(url, file_name, False)

        total_size = int(response.headers.get('content-length', 0))
        downloaded_size = 0

        with open(full_path, 'wb') as f:
            for chunk in response.iter_content(chunk_size=8192):
                if chunk:
                    f.write(chunk)
                    downloaded_size += len(chunk)
                    print(f"\rDownloading... {downloaded_size}/{total_size} bytes", end='')

                    # Check if user wants to exit during download
                    if time_to_exit:
                        print("\nDownload interrupted by user.")
                        return False

        print(f"\nDownload completed: {local_file_name}")
        return True

    except Exception as e:
        print(f"Download failed: {e}")
        return False

# Deleting is irreversible, so make the user type the whole word.
def confirm_destructive(what: str) -> bool:
    print(what)
    try:
        answer = input("This cannot be undone. Type 'yes' to confirm: ").strip()
    except EOFError:
        return False

    if answer != "yes":
        print("Cancelled.")
        return False
    return True

# Delete request: /delete/<name> is a GET, the "delete all" endpoints are POSTs.
# Success is a redirect back to the listing.
def send_delete(request_url: str, use_post: bool, allow_retry: bool = True) -> bool:
    global listed

    try:
        if use_post:
            response = session.post(request_url, timeout=60, allow_redirects=False)
        else:
            response = session.get(request_url, timeout=60, allow_redirects=False)
    except requests.RequestException as e:
        print(f"Request failed: {e}")
        return False

    if response.status_code == 200 and is_login_page(response.text):
        print("The session has expired.")
        if not allow_retry or not do_interactive_login(base_url):
            return False
        return send_delete(request_url, use_post, False)

    # 302 = redirected back to the listing, 200 = plain acknowledgement
    if response.status_code in (200, 302):
        listed = False      # the cached listing is stale now
        return True

    print(f"Delete failed: HTTP {response.status_code}")
    return False

# Download every listed file matching the given extension filter, refreshing
# the listing first when it has not been fetched yet.
def download_all(url: str, matches) -> None:
    if not listed and not directory_listing(f"{url}/list-file"):
        return

    for element in href_elements:
        if matches(decode_url_spaces(element)):
            download_file(url, element)
            if time_to_exit:
                break

# Main function to handle user interaction and media file downloads
def main():
    global download_directory, time_to_exit, base_url

    print("Starting Download Media Files example...")
    print("Press Ctrl+C to exit at any time.")
    print(f"Using payload IP from config: {udp_ip_target}")

    # Setup signal handler for Ctrl+C
    signal.signal(signal.SIGINT, quit_handler)

    if len(sys.argv) == 1:
        print("The download directory is in current folder.")
    else:
        if is_directory_path(sys.argv[1]):
            download_directory = sys.argv[1]
            print(f"The download directory: {download_directory}")
        else:
            print("The download directory is in current folder.")

    base_url = f"http://{udp_ip_target}:8000"
    print(f"Media server URL: {base_url}")

    # Sign in only when the payload actually asks for it.
    if not ensure_authenticated(base_url):
        print("Could not access the media server. Exiting.")
        return

    # Payloads with both internal flash and an SD card let the user pick one.
    choose_storage_source(base_url)

    while not time_to_exit:
        try:
            print("\n----")
            print("Select an option:")
            print("  1. List media files")
            print("  2. Download a Image or a Video")
            print("  3. Download all Images")
            print("  4. Download all Videos")
            print("  5. Delete a Image or a Video")
            print("  6. Delete all Images")
            print("  7. Delete all Videos")
            print("  8. Change storage source (Internal / SD Card)")
            print("  Enter 'q' to quit")
            choice = input("Choice: ").strip().lower()
            print("")

            if choice == "1":
                print("Listing items...")
                if directory_listing(f"{base_url}/list-file"):
                    print("")
                    for element in href_elements:
                        print(decode_url_spaces(element))
                    if not href_elements:
                        print("(no media files on the payload)")

            elif choice == "2":
                if directory_listing(f"{base_url}/list-file"):
                    print("")
                    for element in href_elements:
                        print(decode_url_spaces(element))
                    print("--")
                    name_input = input("Downloading a image or video. Enter the name: ").strip()
                    name_parts = name_input.split()

                    if len(name_parts) != 1:
                        print("Invalid choice. Please try again.")
                    else:
                        download_file(base_url, encode_url_spaces(name_parts[0]))

            elif choice == "3":
                print("Downloading all Images...")
                download_all(base_url, is_image_extension)

            elif choice == "4":
                print("Downloading all Videos...")
                download_all(base_url, is_video_extension)

            elif choice == "5":
                if directory_listing(f"{base_url}/list-file"):
                    print("")
                    for element in href_elements:
                        print(decode_url_spaces(element))
                    if not href_elements:
                        print("(no media files on the payload)")
                        continue
                    print("--")
                    name_input = input("Deleting a image or video. Enter the name: ").strip()
                    name_parts = name_input.split()

                    if len(name_parts) != 1:
                        print("Invalid choice. Please try again.")
                    else:
                        name = name_parts[0]
                        if confirm_destructive(f"Delete '{name}' from the payload?") and \
                                send_delete(f"{base_url}/delete/{encode_url_spaces(name)}", False):
                            print("Deleted.")

            elif choice == "6":
                if confirm_destructive("Delete ALL images on the payload?") and \
                        send_delete(f"{base_url}/delete-all-images", True):
                    print("All images deleted.")

            elif choice == "7":
                if confirm_destructive("Delete ALL videos on the payload?") and \
                        send_delete(f"{base_url}/delete-all-videos", True):
                    print("All videos deleted.")

            elif choice == "8":
                choose_storage_source(base_url)

            elif choice == "q":
                print("Exiting program...")
                break

            else:
                print("Invalid choice. Please try again.")

        except KeyboardInterrupt:
            # This will be caught by the signal handler
            pass
        except EOFError:
            # Handle Ctrl+D
            print("\nEOF detected. Exiting...")
            break

    print("Download media files program finished.")

if __name__ == "__main__":
    main()
