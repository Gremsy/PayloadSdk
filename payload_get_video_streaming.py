import time
import signal
import sys
import threading
from enum import Enum
import os

from libs.payload_sdk import PayloadSdkInterface, payload_status_event_t, param_type, get_stream_sequence_t, camera_cap_flags, video_stream_type
from libs.payload_define import *

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

my_payload = None
time_to_exit = False
stream_uri = ""
is_rtsp_stream = False
video_thread = None
main_pipeline = None
loop = None

my_job = get_stream_sequence_t.CHECK_CAMERA_INFO
time_to_view = 10

# Signal handler for quitting
def quit_handler(sig, frame):
    global my_payload, time_to_exit
    print("\nTERMINATING AT USER REQUEST\n")
    time_to_exit = True

    # Close payload interface
    if my_payload:
        try:
            my_payload.sdkQuit()
            gstreamer_terminate()
        except Exception as e:
            print(f"Error while quitting payload: {e}")

    sys.exit(0)

# Callback function for payload status changes
def onPayloadStatusChanged(event: int, param: list):
    global my_job
    if payload_status_event_t(event) == payload_status_event_t.PAYLOAD_CAM_INFO:
        if int(param[0]) & int(camera_cap_flags.CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM):
            print("   ---> Got payload has streaming video, Check streaming URI")
            my_job = get_stream_sequence_t.CHECK_STREAMING_URI
        else:
            print("   ---> Payload has no streaming video")
            print("It looks like your video streaming setting was DISABLE Auto Connection")
            print("Please enter the Web server and switch Auto Connection to ENABLE. Then try again.")
            my_job = get_stream_sequence_t.IDLE

# Callback function for payload stream changes
def onPayloadStreamChanged(event: int, param_char: str, param_double: list):
    global my_job, stream_uri, is_rtsp_stream
    if payload_status_event_t(event) == payload_status_event_t.PAYLOAD_CAM_STREAMINFO:
        print("   ---> Got streaming information:")
        print(f"   ---> Streaming type: {param_double[0]:.2f}")
        print(f"   ---> Streaming resolution_v: {param_double[1]:.2f}")
        print(f"   ---> Streaming resolution_h: {param_double[2]:.2f}")
        print(f"   ---> Streaming uri: {param_char}")

        if my_job == get_stream_sequence_t.CHECK_STREAMING_URI:
            my_job = get_stream_sequence_t.START_PIPELINE
            is_rtsp_stream = (video_stream_type(param_double[0]) == video_stream_type.VIDEO_STREAM_TYPE_RTSP)
            stream_uri = param_char

def run_video_stream():
    global stream_uri, main_pipeline, loop

    if not is_rtsp_stream or not stream_uri:
        print("No RTSP stream available")
        return

    # Initialize GStreamer
    Gst.init(None)

    # GStreamer pipeline
    gst_pipeline = f"rtspsrc location={stream_uri} latency=0 ! decodebin ! videoconvert ! autovideosink sync=false"
    print(f"Starting GStreamer pipeline: {gst_pipeline}")

    # Create pipeline
    main_pipeline = Gst.parse_launch(gst_pipeline)
    if not main_pipeline:
        print("Failed to create pipeline")
        return

    # Start playing
    main_pipeline.set_state(Gst.State.PLAYING)

    # Create main loop
    loop = GLib.MainLoop()
    try:
        loop.run()
    except Exception as e:
        print(f"Error in GStreamer loop: {e}")

    # Cleanup when loop exits
    main_pipeline.set_state(Gst.State.NULL)

# Start GStreamer pipeline to display video
def gstreamer_start():
    global video_thread
    video_thread = threading.Thread(target=run_video_stream)
    video_thread.daemon = True
    video_thread.start()
    print("GStreamer thread created\n")

# Stop GStreamer pipeline
def gstreamer_terminate():
    global time_to_exit, main_pipeline, loop
    print("Exit GStreamer")
    time_to_exit = True
    if loop:
        loop.quit()
    if main_pipeline:
        main_pipeline.set_state(Gst.State.NULL)
    if video_thread and video_thread.is_alive():
        video_thread.join()

def main():
    global my_payload, my_job, time_to_exit

    print("Starting GetStreaming example...")
    signal.signal(signal.SIGINT, quit_handler)

    # Create payloadsdk object
    my_payload = PayloadSdkInterface()

    # Init payload
    my_payload.sdkInitConnection()
    print("Waiting for payload signal!")

    # Register callback functions
    my_payload.regPayloadStatusChanged(onPayloadStatusChanged)
    my_payload.regPayloadStreamChanged(onPayloadStreamChanged)

    # Check connection
    # my_payload.checkPayloadConnection()

    # Set view source to IR/EO
    my_payload.setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, payload_camera_view_src.PAYLOAD_CAMERA_VIEW_IREO, param_type.PARAM_TYPE_UINT32)
    time.sleep(0.5)

    while not time_to_exit:
        if my_job == get_stream_sequence_t.IDLE:
            print("Program exit.")
            try:
                my_payload.sdkQuit()
            except Exception as e:
                print(f"Error while quitting payload: {e}")
            sys.exit(0)

        elif my_job == get_stream_sequence_t.CHECK_CAMERA_INFO:
            print("Send request to read camera's information")
            my_payload.getPayloadCameraInformation()

        elif my_job == get_stream_sequence_t.CHECK_STREAMING_URI:
            my_payload.getPayloadCameraStreamingInformation()

        elif my_job == get_stream_sequence_t.START_PIPELINE:
            gstreamer_start()
            my_job = get_stream_sequence_t.PIPELINE_RUNNING

        elif my_job == get_stream_sequence_t.PIPELINE_RUNNING:
            while not time_to_exit:
                time.sleep(1)

        time.sleep(1.5)

if __name__ == "__main__":
    main()