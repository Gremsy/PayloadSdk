#include <main.h>
#include <thread>
#include <chrono>


int main(int argc, char* argv[]) {
    
    initPayloadSDKInterface();
    
    initPayloadControlUI();

    return 1;
}

void initPayloadSDKInterface(){
    my_payload = new PayloadSdkInterface(s_conn);
    my_payload->sdkInitConnection();
    my_payload->checkPayloadConnection();

    my_payload->regPayloadStatusChanged(onPayloadStatusChanged);
    my_payload->regPayloadParamChanged(onPayloadParamChanged);

    my_payload->setParamRate(PARAM_EO_ZOOM_LEVEL, 1000);
	my_payload->setParamRate(PARAM_IR_ZOOM_LEVEL, 1000);

    my_payload->setParamRate(PARAM_LRF_RANGE, 100);
    my_payload->setParamRate(PARAM_LRF_OFSET_X, 100);
    my_payload->setParamRate(PARAM_LRF_OFSET_Y, 100);

    my_payload->setParamRate(PARAM_TARGET_COOR_LON, 1000);
    my_payload->setParamRate(PARAM_TARGET_COOR_LAT, 1000);
    my_payload->setParamRate(PARAM_TARGET_COOR_ALT, 1000);

    my_payload->setParamRate(PARAM_CAM_VIEW_MODE, 1000);
    my_payload->setParamRate(PARAM_CAM_REC_SOURCE, 1000);
    my_payload->setParamRate(PARAM_CAM_IR_TYPE, 1000);
    my_payload->setParamRate(PARAM_CAM_IR_PALETTE_ID, 1000);
    
    my_payload->setParamRate(PARAM_GIMBAL_MODE, 1000);

    my_payload->setParamRate(PARAM_PAYLOAD_GPS_LON, 1000);
    my_payload->setParamRate(PARAM_PAYLOAD_GPS_LAT, 1000);
    my_payload->setParamRate(PARAM_PAYLOAD_GPS_ALT, 1000);

    my_payload->setParamRate(PARAM_CAM_IR_FFC_MODE, 1000);

    my_payload->setParamRate(PARAM_IR_TEMP_MAX, 1000);
    my_payload->setParamRate(PARAM_IR_TEMP_MIN, 1000);
    my_payload->setParamRate(PARAM_IR_TEMP_MEAN, 1000);
}

void initPayloadControlUI(){
    auto app = Gtk::Application::create("org.example.camera_ui");
    window = new MainWindow(1920, 1080);
    window->regUICommandChanged(onUICommandChanged);
    app->run(*window);
}

void onUICommandChanged(int event, double* param){
    // printf("got event: %d\n", event);
    switch(event){
        case CAM_VIEW_MODE:{
            if(my_payload != nullptr)
                my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, (int)param[0], PARAM_TYPE_UINT32);
            break;
        }

        case CAM_SOURCE_RECORD:{
            if(my_payload != nullptr)
                my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_RECORD_SRC, (int)param[0], PARAM_TYPE_UINT32);
            break;
        }

        case CAM_CAPTURE:{
            if(my_payload != nullptr)
                my_payload->setPayloadCameraCaptureImage();
            break;
        }

        case CAM_RECORD:{
            if(my_payload != nullptr){
                if(param[0] == 0.0)
                    my_payload->setPayloadCameraRecordVideoStart();
                else if(param[0] == 1.0)
                    my_payload->setPayloadCameraRecordVideoStop();
            }
            break;
        }

        case CAM_EO_SPEED_ZOOM:{
            if(my_payload != nullptr)
                my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_EO_ZOOM_SPEED, (int)param[0], PARAM_TYPE_UINT32);
            break;
        }

        case CAM_ZOOM_CONTINIOUS:{
            if(my_payload != nullptr)
                my_payload->setCameraZoom(ZOOM_TYPE_CONTINUOUS, param[0]);
            break;
        }

        case CAM_ZOOM_STEP:{
            if(my_payload != nullptr)
                my_payload->setCameraZoom(ZOOM_TYPE_STEP, param[0]);
            break;
        }

        case CAM_ZOOM_RANGE:{
            if(my_payload != nullptr)
                my_payload->setCameraZoom(ZOOM_TYPE_RANGE, param[0]);
            break;
        }

        case CAM_EO_SPEED_FOCUS:{
            if(my_payload != nullptr)
                my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_EO_FOCUS_SPEED, (int)param[0], PARAM_TYPE_UINT32);
            break;
        }

        case CAM_FOCUS_AUTO:{
            if(my_payload != nullptr)
                my_payload->setCameraFocus(FOCUS_TYPE_AUTO);
            break;
        }

        case CAM_FOCUS_CONTINIOUS:{
            if(my_payload != nullptr)
                my_payload->setCameraFocus(FOCUS_TYPE_CONTINUOUS, param[0]);
            break;
        }

        case CAM_AE_MODE:{
            if(my_payload != nullptr)
                my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_AUTO_EXPOSURE, (int)param[0], PARAM_TYPE_UINT32);
            break;
        }

        case CAM_SHUTTER:{
            if(my_payload != nullptr)
                my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED, (int)param[0], PARAM_TYPE_UINT32);
            break;
        }

        case CAM_IRIS:{
            if(my_payload != nullptr)
                my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_APERTURE_VALUE, (int)param[0], PARAM_TYPE_UINT32);
            break;
        }

        case CAM_GAIN:{
            if(my_payload != nullptr)
                my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_EO_GAIN_LS, (int)param[0], PARAM_TYPE_UINT32);
            break;
        }

        case CAM_WHITE_BALANCE:{
            if(my_payload != nullptr)
                my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE, (int)param[0], PARAM_TYPE_UINT32);
            break;
        }

        case CAM_WHITE_BALANCE_TRIGGER:{
            if(my_payload != nullptr)
                my_payload->setPayloadCameraWBTrigg();
            break;
        }

        case CAM_IR_PALETTE:{
            if(my_payload != nullptr)
                my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_IR_PALETTE, (int)param[0], PARAM_TYPE_UINT32);
            break;
        }

        case CAM_LRF_MODE:{
            if(my_payload != nullptr)
                my_payload->setPayloadCameraParam(PAYLOAD_LRF_MODE, (int)param[0], PARAM_TYPE_UINT32);
            break;
        }

        case CAM_OSD_MODE:{
            if(my_payload != nullptr)
                my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_OSD_MODE, (int)param[0], PARAM_TYPE_UINT32);
            break;
        }

        case CAM_IMAGE_FLIP:{
            if(my_payload != nullptr)
                my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_FLIP, (int)param[0], PARAM_TYPE_UINT32);
            break;
        }

        case GIMBAL_MODE:{
            if(my_payload != nullptr)
                my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, (int)param[0], PARAM_TYPE_UINT32);
            
            break;
        }

        case GIMBAL_CONTROL_TILT:{
            if(my_payload != nullptr)
                my_payload->setGimbalSpeed(param[0], 0 , 0, INPUT_SPEED);

            break;
        }

        case GIMBAL_CONTROL_PAN:{
            if(my_payload != nullptr)
                my_payload->setGimbalSpeed(0, 0 , param[0], INPUT_SPEED);

            break;
        }

        case GIMBAL_CONTROL_ANGLE:{
            if(my_payload != nullptr)
                my_payload->setGimbalSpeed(param[0], param[1] , param[2], INPUT_ANGLE);

            break;
        }

        case QUERY_PAYLOAD_PARAM:{
            if(my_payload != nullptr)
                my_payload->getPayloadCameraSettingList();

            break;
        }
        
        default: break;
    }
}

void onPayloadParamChanged(int event, char* param_char, double* param){
	// printf("%s %d \n", __func__, event);
	switch(event){
	case PAYLOAD_CAM_PARAMS:{
		// param[0]: param_index
		// param[1]: value
		// printf(" --> Param_id: %s, value: %.2f\n", param_char, param[1]);
        if (window != nullptr) {
            window->update_payload_param(param_char, param[1]);
        }
		break;
	}
	
	default: break;
	}
}


void onPayloadStatusChanged(int event, double* param){
	// printf("%s %d \n", __func__, event);
	switch(event){
	case PAYLOAD_CAM_STORAGE_INFO:{
		// param[0]: total_capacity
		// param[1]: used_capacity
		// param[2]: available_capacity
		// param[3]: status

        if (window != nullptr) {
            window->update_storage_info((int)param[3], param[0], param[1], param[2]);
        }

		break;
	}
    case PAYLOAD_CAM_CAPTURE_STATUS:{
        if (window != nullptr) {
            window->update_capture_info((int)param[0],(int)param[1], (int)param[2],(int)param[3]);
        }

		break;
	}

    case PAYLOAD_GB_ATTITUDE:{
		// param[0]: pitch
		// param[1]: roll
		// param[2]: yaw

		// printf("Pich: %.2f - Roll: %.2f - Yaw: %.2f\n", param[0], param[1], param[2]);
        if (window != nullptr) {
            window->update_gimbal_attitude(param[0],param[1], param[2]);
        }
		break;
	}

    case PAYLOAD_PARAMS:{
		// param[0]: param index
		// param[1]: value
        if (window != nullptr) {
            window->update_payload_status(param);
        }
	}

	default: break;
        

	}


}