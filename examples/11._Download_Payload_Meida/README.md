# Download Media Files on Payload VIO and ZIO

## Hardware Requirements:
This example uses Payload VIO or ZIO with the following version requirements:
- VIO version: 1.0.4 or higher
- ZIO version: 2.0.0 or higher

## Usage:
To download media files from the Payload VIO or ZIO system, follow the steps below:

1. Build Gremsy's Payloads.
2. Go to build directory.
3. Run: ```./examples/11._Download_Payload_Media/DownloadPayloadMedia xxx.xxx.xxx.xxx /path/to/download/folder```

- If the ```/path/to/download/folder```  argument is not passed, the program will use the default download folder located in the ./build directory.
- If the ```Payload IP address``` (`xxx.xxx.xxx.xxx`) is not passed, the program will prompt you to enter it.

4. Input the number of the option:
    ```
    Select an option:
    1. List meida files
    2. Download a Image or a Video 
    3. Download all Images
    4. Download all Videos
    Enter 'q' to quit