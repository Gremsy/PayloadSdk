## PayloadSDK Functions
- [Table of contents](#table-of-contents)

- [Version history](#version-history)

- [Connection functions](#connection-functions)
    - [sdkInitConnection](#sdkinitconnection)
    - [checkPayloadConnection](#checkpayloadconnection)
    - [sdkQuit](#sdkquit)

- [Camera Control functions](#camera-control-functions)
    - [setPayloadCameraParam](#setpayloadcameraparam)
    - [setPayloadCameraMode](#setpayloadcameramode)
    - [setPayloadCameraCaptureImage](#setpayloadcameracaptureimage)
    - [setPayloadCameraStopImage](#setpayloadcamerastopimage)
    - [setPayloadCameraRecordVideoStart](#setpayloadcamerarecordvideostart)
    - [setPayloadCameraRecordVideoStop](#setpayloadcamerarecordvideostop)
    - [setCameraZoom](#setcamerazoom)
    - [setCameraFocus](#setcamerafocus)
    - [getPayloadCameraSettingList](#getpayloadcamerasettinglist)
    - [getPayloadCameraSettingByID](#getpayloadcamerasettingbyid)
    - [getPayloadCameraSettingByIndex](#getpayloadcamerasettingbyindex)
    - [getPayloadCameraMode](#getpayloadcameramode)
    - [getPayloadCameraInformation](#getpayloadcamerainformation)
    - [getPayloadCameraStreamingInformation](#getpayloadcamerastreaminginformation)
    - [getPayloadStorage](#getpayloadstorage)
    - [getPayloadCaptureStatus](#getpayloadcapturestatus)
    - [setParamRate](#setparamrate)

- [Gimbal Control functions](#gimbal-control-functions)
    - [setPayloadGimbalParamByID](#setpayloadgimbalparambyid)
    - [setGimbalSpeed](#setgimbalspeed)
    - [getPayloadGimbalSettingList](#getpayloadgimbalsettinglist)
    - [getPayloadGimbalSettingByID](#getpayloadgimbalsettingbyid)
    - [getPayloadGimbalSettingByIndex](#getpayloadgimbalsettingbyindex)
    - [sendPayloadGimbalCalibGyro](#sendpayloadgimbalcalibgyro)
    - [sendPayloadGimbalCalibAccel](#sendpayloadgimbalcalibaccel)
    - [sendPayloadGimbalCalibMotor](#sendpayloadgimbalcalibmotor)
    - [sendPayloadGimbalSearchHome](#sendpayloadgimbalsearchhome)
    - [sendPayloadGimbalAutoTune](#sendpayloadgimbalautotune)

<div style="page-break-after: always"></div>

- [FFC Control functions](#ffc-control-functions)
    - [setPayloadCameraFFCMode](#setpayloadcameraffcmode)
    - [setPayloadCameraFFCTrigg](#setpayloadcameraffctrigg)

- [GPS functions](#gps-functions)
    - [sendPayloadGPSRawInt](#sendpayloadgpsrawint)
    - [sendPayloadGPSPosition](#sendpayloadgpsposition)

- [System Time function](#system-time-function)
    - [sendPayloadSystemTime](#sendpayloadsystemtime)

- [SD Card function](#sd-card-function)
    - [setFormatSDCard](#setformatsdcard)

- [Object Tracking function](#object-tracking-function)
    - [setPayloadObjectTrackingParams](#setpayloadobjecttrackingparams)

- [Callback functions](#callback-functions)
    - [regPayloadStatusChanged](#regpayloadstatuschanged)
    - [regPayloadParamChanged](#regpayloadparamchanged)
    - [regPayloadStreamChanged](#regpayloadstreamchanged)

<div style="page-break-after: always"></div>

## Version history

|   Version |   Publish date    |   Description                 |
|   -       |   -               |   -                           |
|   v1.0.0  |   05/23/2025      |   Add format SD card function |

<div style="page-break-after: always"></div>

## Connection functions

### sdkInitConnection

- Brief: Initialize the connection to the payload via UDP or UART.

| Param | Type | Range | Default Value | Description |
|-------|------|-------|---------------|-------------|
| None  | None | None  | None          | None        |

- Return: True if connection successful, False otherwise (bool)

### checkPayloadConnection

- Brief: Check the connection to the payload within a specified timeout period.

| Param   | Type  | Range | Default Value | Description        |
|---------|-------|-------|---------------|--------------------|
| timeout | float | > 0   | 5.0           | Timeout in seconds |

- Return: True if connection successful, False otherwise (bool)

### sdkQuit

- Brief: Close the connection and stop the data receiving thread.

| Param | Type | Range | Default Value | Description |
|-------|------|-------|---------------|-------------|
| None  | None | None  | None          | None        |

- Return: None

<div style="page-break-after: always"></div>

## Camera Control functions

### setPayloadCameraParam

- Brief: Set the parameter value for the payload's camera.

| Param          | Type  | Range                                             | Default Value | Description                   |
|----------------|-------|---------------------------------------------------|---------------|-------------------------------|
| param_id       | str   | None  | None          | Parameter ID              |
| param_value    | int   | None  | None          | Parameter value           |
| param_type     | int   | (1) MAV_PARAM_TYPE_UINT8 <br> (2) MAV_PARAM_TYPE_INT8 <br> (3) MAV_PARAM_TYPE_UINT16 <br> (4) MAV_PARAM_TYPE_INT16 <br> (5) MAV_PARAM_TYPE_UINT32 <br> (6) MAV_PARAM_TYPE_INT32 <br> (7) MAV_PARAM_TYPE_UINT64 <br> (8) MAV_PARAM_TYPE_INT64 <br> (9) MAV_PARAM_TYPE_REAL32 <br> (10)MAV_PARAM_TYPE_REAL64  | None    | Data type of the parameter |

- Return: None

### setPayloadCameraMode

- Brief: Set the operating mode for the camera

| Param          | Type  | Range | Default Value | Description                   |
|----------------|-------|-------|---------------|-------------------------------|
| mode           | int   | (0) CAMERA_MODE_IMAGE <br> (1) CAMERA_MODE_VIDEO | None | Camera mode |

- Return: None

### setPayloadCameraCaptureImage

- Brief: Request the camera to capture an image.

| Param      | Type | Range | Default Value | Description                          |
|------------|------|-------|---------------|--------------------------------------|
| interval_s | int  | >= 0  | 0             | Interval between captures in seconds |

- Return: None

<div style="page-break-after: always"></div>

### setPayloadCameraStopImage

- Brief: Stop the interval image capturing process.

| Param | Type | Range | Default Value | Description |
|-------|------|-------|---------------|-------------|
| None  | None | None  | None          | None        |

- Return: None

### setPayloadCameraRecordVideoStart

- Brief: Start video recording.

| Param | Type | Range | Default Value | Description |
|-------|------|-------|---------------|-------------|
| None  | None | None  | None          | None        |

- Return: None

### setPayloadCameraRecordVideoStop

- Brief: Stop video recording.

| Param | Type | Range | Default Value | Description |
|-------|------|-------|---------------|-------------|
| None  | None | None  | None          | None        |

- Return: None

### setCameraZoom

- Brief: Adjust the camera's zoom level.

| Param          | Type  | Range | Default Value | Description             |
|----------------|-------|-------|---------------|-------------------------|
| zoom_type      | float | (0) ZOOM_TYPE_STEP <br> (1) ZOOM_TYPE_CONTINUOUS <br> (2) ZOOM_TYPE_RANGE | None   | Type of zoom                         |
| zoom_value     | float | zoom_type equal ZOOM_TYPE_STEP and equal ZOOM_TYPE_CONTINUOUS <br> (-1) ZOOM_OUT <br> (0) ZOOM_STOP <br> (1) ZOOM_IN  | None   | Zoom value              |
| zoom_value     | float | zoom_type equal ZOOM_TYPE_RANGE <br> from 0.0% to 100.00%  | None   | Zoom value              |

- Return: None

<div style="page-break-after: always"></div>

### setCameraFocus

- Brief: Adjust the camera's focus.

| Param       | Type  | Range | Default Value | Description   |
|-------------|-------|-------|---------------|---------------|
| focus_type  | float | (1) ZOOM_TYPE_CONTINUOUS <br> (4) FOCUS_TYPE_AUTO  | None          | Type of focus |
| focus_value | float | zoom_type equal ZOOM_TYPE_CONTINUOUS <br> (-1) FOCUS_OUT <br> (0) FOCUS_STOP <br> (1) FOCUS_IN <br> (2) FOCUS_AUTO  | 0             | Focus value   |

- Return: None

### getPayloadCameraSettingList

- Brief: Request the list of all camera parameters.

| Param | Type | Range | Default Value | Description |
|-------|------|-------|---------------|-------------|
| None  | None | None  | None          | None        |

- Return: None

### getPayloadCameraSettingByID

- Brief: Request the value of a specific camera parameter by ID.

| Param    | Type | Range | Default Value | Description  |
|----------|------|-------|---------------|--------------|
| param_id | str  | None  | None          | Parameter ID |

- Return: None

### getPayloadCameraSettingByIndex

- Brief: Request the value of a camera parameter by index.

| Param | Type | Range | Default Value | Description     |
|-------|------|-------|---------------|-----------------|
| idx   | int  | None  | None          | Parameter index |

- Return: None

<div style="page-break-after: always"></div>

### getPayloadCameraMode

- Brief: Request the current mode of the camera.

| Param | Type | Range | Default Value | Description |
|-------|------|-------|---------------|-------------|
| None  | None | None  | None          | None        |

- Return: None

### getPayloadCameraInformation

- Brief: Request detailed information about the camera.

| Param | Type | Range | Default Value | Description |
|-------|------|-------|---------------|-------------|
| None  | None | None  | None          | None        |

- Return: None

### getPayloadCameraStreamingInformation

- Brief: Request information about the camera's video stream.

| Param | Type | Range | Default Value | Description |
|-------|------|-------|---------------|-------------|
| None  | None | None  | None          | None        |

- Return: None

### getPayloadStorage

- Brief: Request information about the camera's storage (SD card).

| Param | Type | Range | Default Value | Description |
|-------|------|-------|---------------|-------------|
| None  | None | None  | None          | None        |

- Return: None

### getPayloadCaptureStatus

- Brief: Request the current capture status of the camera.

| Param | Type | Range | Default Value | Description |
|-------|------|-------|---------------|-------------|
| None  | None | None  | None          | None        |

- Return: None

<div style="page-break-after: always"></div>

### setParamRate

- Brief: Set the message rate for a specific parameter.

| Param        | Type | Range | Default Value | Description |
|--------------|------|-------|---------------|-------------|
| param_index  | int  | None  | Valid indices from PAYLOAD_PARAMS          | Index of the parameter        |
| time_ms      | int  | None  | ≥ 0          | Time interval between messages in milliseconds              |

- Return: None

<div style="page-break-after: always"></div>

## Gimbal Control functions

### setPayloadGimbalParamByID

- Brief: Set the value of a gimbal parameter by ID.

| Param       | Type  | Range | Default Value | Description     |
|-------------|-------|-------|---------------|-----------------|
| param_id    | str   | None  | None          | Parameter ID    |
| param_value | float | None  | None          | Parameter value |

- Return: None

### setGimbalSpeed

- Brief: Set the speed or angle for move gimbal

| Param     | Type         | Range        | Default Value | Description                    |
|-----------|--------------|--------------|---------------|--------------------------------|
| spd_pitch | float        | -90 to 90    | None          | Pitch speed/angle in degrees   |
| spd_roll  | float        | -180 to 180  | None          | Roll speed/angle in degrees    |
| spd_yaw   | float        | -180 to 180  | None          | Yaw speed/angle in degrees     |
| mode      | input_mode_t | (1)INPUT_ANGLE <br> (2)INPUT_SPEED   | None | Input mode              |

- Return: None

### getPayloadGimbalSettingList
- Brief: Request the list of all gimbal parameters

| Param | Type | Range | Default Value | Description |
|-------|------|-------|---------------|-------------|
| None  | None | None  | None          | None        |

- Return: None

### getPayloadGimbalSettingByID

- Brief: Request the value of a specific gimbal parameter by ID.

| Param    | Type | Range | Default Value | Description  |
|----------|------|-------|---------------|--------------|
| param_id | str  | None  | None          | Parameter ID |

- Return: None

<div style="page-break-after: always"></div>

### getPayloadGimbalSettingByIndex

- Brief: Request the value of a gimbal parameter by index.

| Param | Type | Range | Default Value | Description     |
|-------|------|-------|---------------|-----------------|
| idx   | int  | None  | None          | Parameter index |

- Return: None

### sendPayloadGimbalCalibGyro

- Brief: Send command to calibrate the gimbal's gyro.

| Param | Type | Range | Default Value | Description |
|-------|------|-------|---------------|-------------|
| None  | None | None  | None          | None        |

- Return: None

### sendPayloadGimbalCalibAccel

- Brief: Send command to calibrate the gimbal's accelerometer.

| Param | Type | Range | Default Value | Description |
|-------|------|-------|---------------|-------------|
| None  | None | None  | None          | None        |

- Return: None

### sendPayloadGimbalCalibMotor

- Brief: Send command to calibrate the gimbal's motor.

| Param | Type | Range | Default Value | Description |
|-------|------|-------|---------------|-------------|
| None  | None | None  | None          | None        |

- Return: None

### sendPayloadGimbalSearchHome

- Brief: Send command to search for the gimbal's home position.

| Param | Type | Range | Default Value | Description |
|-------|------|-------|---------------|-------------|
| None  | None | None  | None          | None        |

- Return: None

<div style="page-break-after: always"></div>

### sendPayloadGimbalAutoTune

- Brief: Send command to auto-tune the gimbal.

| Param  | Type | Range | Default Value | Description                      |
|--------|------|-------|---------------|----------------------------------|
| status | bool | True/False | None     | True to enable, False to disable |

- Return: None

<div style="page-break-after: always"></div>

## FFC Control Functions

### setPayloadCameraFFCMode

- Brief: Set the FFC mode for the camera.

| Param | Type | Range | Default Value | Description                   |
|-------|------|-------|---------------|-------------------------------|
| mode  | int  | 0-1   | None          | FFC mode (0: Manual, 1: Auto) |

- Return: None

### setPayloadCameraFFCTrigg

- Brief: Trigger FFC calibration for the camera.

| Param | Type | Range | Default Value | Description |
|-------|------|-------|---------------|-------------|
| None  | None | None  | None          | None        |

- Return: None

<div style="page-break-after: always"></div>

## GPS functions

### sendPayloadGPSRawInt

- Brief: Send the GPS RAW information to the payload.

| Param     | Type                        | Range | Default Value | Description                 |
|-----------|-----------------------------|-------|---------------|-----------------------------|
| gps_raw  | mavlink_gps_raw_int_t | None  | None        | GPS raw data structure |

- Return: None

### sendPayloadGPSPosition

- Brief: Send GPS position information to the payload.

| Param     | Type                        | Range | Default Value | Description                 |
|-----------|-----------------------------|-------|---------------|-----------------------------|
| gps_data  | mavlink_global_position_int_t | None  | None        | GPS position data structure |

- Return: None

<div style="page-break-after: always"></div>

## System Time function

### sendPayloadSystemTime

- Brief: Send system time information to the payload.

| Param         | Type                  | Range | Default Value | Description                |
|---------------|-----------------------|-------|---------------|----------------------------|
| sys_time_data | mavlink_system_time_t | None  | None          | System time data structure |

- Return: None

<div style="page-break-after: always"></div>

## SD Card function

### setFormatSDCard

- Brief: Format the camera's SD card.

| Param | Type | Range | Default Value | Description |
|-------|------|-------|---------------|-------------|
| None  | None | None  | None          | None        |

- Return: None

<div style="page-break-after: always"></div>

## Object Tracking function

### setPayloadObjectTrackingParams

- Brief: Set parameters for object tracking functionality.

| Param | Type  | Range | Default Value | Description      |
|-------|-------|-------|---------------|------------------|
| cmd   | float | (0)TRACK_IDLE <br> (1)TRACK_ACT <br> (2)TRACK_LOST  | None          | Tracking command |
| pos_x | float | None  | 960           | X position       |
| pos_y | float | None  | 540           | Y position       |

- Return: None

<div style="page-break-after: always"></div>

## Callback functions

### regPayloadStatusChanged
- Brief: Register a callback to receive notifications when the payload status changes.

| Param    | Type  | Range | Default Value | Description |
|----------|-------|-------|---------------|-------------|
| callback | Callable | None | None | Function that takes payload_status_event_t and List[float] as parameters |

- Return: None

### regPayloadParamChanged
- Brief: Register a callback to receive notifications when a payload parameter changes.

| Param    | Type  | Range | Default Value | Description |
|----------|-------|-------|---------------|-------------|
| callback | Callable | None | None | Function that takes payload_status_event_t, str, and List[float] as parameters |

- Return: None

### regPayloadStreamChanged
- Brief: Register a callback to receive notifications when the payload's video stream information changes.

| Param    | Type  | Range | Default Value | Description |
|----------|-------|-------|---------------|-------------|
| callback | Callable | None | None | Function that takes payload_status_event_t, str, and List[float] as parameters |

- Return: None