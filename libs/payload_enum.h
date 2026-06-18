#ifndef PAYLOADSDK_ENUM_H_
#define PAYLOADSDK_ENUM_H_

enum input_mode_t {
    INPUT_ANGLE = 1,
    INPUT_SPEED = 2
};

enum ffc_mode_t {
	FFC_MODE_MANUAL=0,
	FFC_MODE_AUTO,
	FFC_MODE_END
};

enum Camera_Zoom_Value
{
   ZOOM_OUT  = -1,
   ZOOM_STOP = 0, 
   ZOOM_IN   = 1,
};

enum Camera_Focus_Value
{
   FOCUS_OUT  = -1,
   FOCUS_STOP = 0, 
   FOCUS_IN   = 1,
   FOCUS_AUTO,
};

#endif