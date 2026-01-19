#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "uvc_camera.h"
#include "opencv_display.h"
#include "cmd.h"
#include "libiruart.h"
#include "temp_measure.h"
#include "main_system.h"

extern StreamFrameInfo_t stream_frame_info;

static void* threadEntry(void* arg);

int processMain(config& config_obj, mainSystem* worker);


