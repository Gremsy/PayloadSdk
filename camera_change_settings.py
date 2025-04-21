import time
import signal
import sys

from libs.payload_sdk import PayloadSdkInterface, param_type, payload_status_event_t, PAYLOAD_TYPE
from libs.payload_define import *

my_payload = None

# Signal handler for quitting
def quit_handler(sig, frame):
    global my_payload
    print("\nTERMINATING AT USER REQUEST")

    # Close payload interface
    try:
        my_payload.sdkQuit()
    except Exception as e:
        print(f"Error while quitting payload: {e}")

    # End program    
    sys.exit(0)

# Callback function for payload param changes
def onPayloadParamChanged(event: int, param_char: str, param: list):
    
    if payload_status_event_t(event) == payload_status_event_t.PAYLOAD_CAM_PARAMS:
        # param[0]: param_index
		# param[1]: value
        print(f" --> Param_id: {param_char}, value: {param[1]:.2f}")

def main():
    global my_payload

    print("Starting SetPayloadSettings example...\n")
    signal.signal(signal.SIGINT, quit_handler)
    
    # Create payloadsdk object
    my_payload = PayloadSdkInterface()

    # Init payload
    my_payload.sdkInitConnection()
    print("Waiting for payload signal!")
    
    # Register callback function
    my_payload.regPayloadParamChanged(onPayloadParamChanged)
    
    # Check connection
    my_payload.checkPayloadConnection()
    
    # Change setting of RC_MODE to STANDARD
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_RC_MODE, payload_camera_rc_mode.PAYLOAD_CAMERA_RC_MODE_STANDARD, param_type.PARAM_TYPE_UINT32) 
    
    # Change setting of OSD_MODE to STATUS to enable viewing of the zoom factor
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_OSD_MODE , payload_camera_osd_mode.PAYLOAD_CAMERA_VIDEO_OSD_MODE_STATUS, param_type.PARAM_TYPE_UINT32)  
    
    print("------------------------> Init values \n")
    # Request to read all settings of the payload and then check the RC_MODE setting
    my_payload.getPayloadCameraSettingList()
    time.sleep(3)  

    print("\nChange some params\n")
    # Change zoom mode to SuperResolution
    if PAYLOAD_TYPE in ["VIO", "ZIO"]:
        my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_MODE, payload_camera_video_zoom_mode.PAYLOAD_CAMERA_VIDEO_ZOOM_MODE_SUPER_RESOLUTION, param_type.PARAM_TYPE_UINT32) 
        time.sleep(3)  

    # Request to read all settings of the payload to verify the changes
    print("------------------------> Changed values \n")
    my_payload.getPayloadCameraSettingList()
    time.sleep(3)  

    # Close payload interface
    try:
        my_payload.sdkQuit()
    except Exception as e:
        print(f"Error while quitting payload: {e}")

if __name__ == "__main__":
    main()