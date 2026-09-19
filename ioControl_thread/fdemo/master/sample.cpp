#include "sample.h"
#include "config.h"

StreamFrameInfo_t stream_frame_info;

static void* threadEntry(void* arg){
    mainSystem* self = static_cast<mainSystem*>(arg);
	self->run();
	return nullptr;
}

int processMain(config& config_obj, mainSystem* worker)
{
    single_config product_config;
    if (false == config_obj.get_config(product_config))
    {
        printf("can't find config\n");
        return -1;
    }

    printf("uart cmd and usb image sample start\n");
    printf("Depends lib version:\n");
    printf("version:%s\n", ircmd_version());
    printf("version:%s\n", ircam_version());
    printf("version:%s\n", iruvc_version());

    printf("set libs log level:\n");
    ircmd_log_register(IRCMD_LOG_ERROR, NULL,NULL);
    iruvc_log_register(IRUVC_LOG_ERROR,NULL,NULL);
    iruart_log_register(IRUART_LOG_ERROR,NULL,NULL);
    ircam_log_register(IRCAM_LOG_ERROR,NULL, NULL);

    printf("init uvc stream handle\n");
    int fd;
    int ret;
    memset(&stream_frame_info, 0, sizeof(stream_frame_info) - sizeof(product_config));
    stream_frame_info.product_config = product_config;

    printf("init stream handle\n");
    ir_video_handle_create(&ir_image_video_handle);//创建video_handle

    IruvcHandle_t* iruvc_handle = NULL;
    iruvc_handle = (IruvcHandle_t*)iruvc_camera_handle_create(ir_image_video_handle);//创建uvc_handle
    stream_frame_info.image_driver_handle = iruvc_handle;
    IruvcDevParam_t dev_param;
    memset(&dev_param, 0, sizeof(IruvcDevParam_t));

    dev_param.pid = product_config.camera.uvc_stream_conf.dev_info.pid;
    dev_param.vid = product_config.camera.uvc_stream_conf.dev_info.vid;
    dev_param.same_idx = product_config.camera.uvc_stream_conf.dev_info.same_id;

    stream_frame_info.image_dev_params = &dev_param;
    ret = ir_image_video_handle->ir_video_open(stream_frame_info.image_driver_handle, &dev_param);
    printf("open video : %d\n", ret);
    ret = ir_image_video_handle->ir_video_init(stream_frame_info.image_driver_handle, &dev_param);
    printf("init video : %d\n", ret);
    IruvcCamStreamParams_t params;
    memset(&params, 0, sizeof(params));
    stream_frame_info.stream_config = &params;

    printf("init control handle\n");
    IrControlHandle_t* ir_control_handle = NULL;
    ir_control_handle_create(&ir_control_handle);
    if (product_config.control.is_usb_control || product_config.control.is_i2c_usb_control)
    {
        if (product_config.control.is_usb_control)
        {
            iruvc_usb_handle_create_with_exist_instance(ir_control_handle, iruvc_handle);
        }
        else
        {
            iruvc_i2c_usb_handle_create_with_exist_instance(ir_control_handle, iruvc_handle);
        }

        do
        {
            if (dev_param.pid == product_config.control.usb_param.pid
                && dev_param.vid == product_config.control.usb_param.vid
                && dev_param.same_idx == product_config.control.usb_param.same_id)
            {
                break;
            }
            IruvcDevParam_t control_dev_param;
            memset(&control_dev_param, 0, sizeof(IruvcDevParam_t));
            control_dev_param.pid = product_config.control.usb_param.pid;
            control_dev_param.vid = product_config.control.usb_param.vid;
            control_dev_param.same_idx = product_config.control.usb_param.same_id;
            ret = ir_control_handle->ir_control_open(iruvc_handle, &control_dev_param);
            printf("open control node: %d\n", ret);
        } while (0);

        stream_frame_info.ircmd_handle = ircmd_create_handle(ir_control_handle);
    }
    else if (product_config.control.is_uart_control)
    {
        IruartHandle_t* iruart_handle = NULL;
        iruart_handle = iruart_handle_create(ir_control_handle);
        IrcmdHandle_t* cmd_handle = NULL;
        ret = search_com(ir_control_handle, &cmd_handle, product_config.control.uart_param.com_index);
        if (ret != 0)
        {
            printf("fail to find available com\n");
            return -1;
        }
        stream_frame_info.ircmd_handle = cmd_handle;
    }


    if (!product_config.camera.is_auto_image)
    {
        ret = basic_video_stream_continue(stream_frame_info.ircmd_handle);
        printf("basic_video_stream_continue:%d\n", ret);    //control mipi start
    }

    load_stream_frame_info(&stream_frame_info, false, true);
    init_pthread_sem();
    pthread_t stream_thread, display_thread, sys_thread, temp_thread;
    pthread_create(&stream_thread, NULL, uvc_stream_function, &stream_frame_info);
    pthread_create(&display_thread, NULL, opencv_display_function, &stream_frame_info);
    if (product_config.camera.open_temp_measure)
    {
        pthread_create(&temp_thread, NULL, temp_measure_function, &stream_frame_info);
    }
    pthread_create(&sys_thread, NULL, threadEntry, (void*)worker);

    printf("in streaming\n");
    pthread_join(stream_thread, NULL);
    printf("stop stream\n");
    if (product_config.camera.open_temp_measure)
    {
        pthread_join(temp_thread, NULL);
    }
    pthread_join(sys_thread, NULL);
    pthread_join(display_thread, NULL);
    destroy_pthread_sem();
    destroy_data_demo(&stream_frame_info);
    
    return 0;
}
