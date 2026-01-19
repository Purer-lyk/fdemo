#ifndef _LIBIRCMD_TEST_H_
#define _LIBIRCMD_TEST_H_

#include "libircmd.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TEST_FLASH_READ_FILE_NAME	"dataread.bin"
#define TEST_FLASH_RW_BUFFER		4096

/**
 * @brief Open the file in selected mode and get the length according file_name
 *
 * @param[in] handle ircmd's handle
 * @param[in] file_name the name of the file
 * @param[in] file_length size of actual files
 * @param[out] file_id unique identification name of the file
 * @param[out] storage_length size of storage space for files
 *
 * @return see IrlibError_e
*/
DLLEXPORT IrlibError_e adv_cfg_file_open(IrcmdHandle_t* handle, char* file_name, \
    uint32_t file_length, int* file_id, uint32_t* storage_length);


/**
 * @brief read data from flash to fill the file
 *
 * @param[in] handle ircmd's handle
 * @param[in] file_id unique identification name of the file
 * @param[out] data storage of read data
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_cfg_file_read(IrcmdHandle_t* handle, int file_id, void* data);


/**
 * @brief write data of the file to flash
 *
 * @param[in] handle ircmd's handle
 * @param[in] file_id unique identification name of the file
 * @param[in] data storage of data to be written
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_cfg_file_write(IrcmdHandle_t* handle, int file_id, void* data);

/**
 * @brief write data of the file to flash by multipart
 *
 * @param[in] handle ircmd's handle
 * @param[in] file_id unique identification name of the file
 * @param[in] data storage of data to be written
 * @param[in] length length of data
 * @param[in] offset data address offset
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_cfg_file_write_multipart(IrcmdHandle_t* handle, int file_id, void* data, int length, uint32_t offset);

/**
 * @brief Close the file
 *
 * @param[in] handle ircmd's handle
 * @param[in] file_id unique identification name of the file
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_cfg_file_close(IrcmdHandle_t* handle, int file_id);


/**
 * @brief Close all the file before upgrade config data
 *
 * @param[in] handle ircmd's handle
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_cfg_file_close_all(IrcmdHandle_t* handle);


/**
 * @brief Perform a file offset
 *
 * @param[in] handle ircmd's handle
 * @param[in] addr offset operation's start position
 * @param[in] length number of bytes for offset operation
 *
 * @return see IrlibError_e
 */
#ifndef COMMAND_IN_DEVELOPING
DLLEXPORT IrlibError_e adv_cfg_file_shift(IrcmdHandle_t* handle, uint32_t addr, uint32_t length);
#endif


/**
 * @brief Fill crc value of the file to file_info.bin
 *
 * @param[in] handle ircmd's handle
 * @param[in] file_id unique identification name of the file
 * @param[in] data original data of the file
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_cfg_file_modify_crc(IrcmdHandle_t* handle, int file_id, void* data);


/**
 * @brief Set device's information
 *
 * @param[in] handle ircmd's handle
 * @param[in] device_info_type the device's information type, see basic_dev_info_type_e
 * @param[in] data[100] the device's information data
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_device_info_set(IrcmdHandle_t* handle, int device_info_type, char* data);


/**
 * @brief Enter file update mode
 * During file update mode, the last two fields in the whole package version read as FF.FF
 *
 * @param[in] handle ircmd's handle
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_enter_file_update_mode(IrcmdHandle_t* handle);


/**
 * @brief Leave file update mode
 * After leaving file update mode, the last two fields in the whole package version is updated to the current data version of the device
 *
 * @param[in] handle ircmd's handle
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_leave_file_update_mode(IrcmdHandle_t* handle);


/**
 * @brief Get video driver's status
 *
 * @param[in] handle ircmd's handle
 * @param[out] status the status of video driver, 0:BUSY 1:IDLE
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_video_driver_status_get(IrcmdHandle_t* handle, int* status);

/**
 * @brief Get device status
 *
 * @param[in] handle ircmd's handle
 * @param[out] status 0:Unknow status,1:Rom code,2:Cache Code
 *
 * @return see IrcmdError_e
 */
DLLEXPORT IrlibError_e adv_detect_device_status_get(IrcmdHandle_t* handle, int* status);

/**
 * Not allow to provide this interface to customers, only for internal firmware development/production calibration
 * Can only read data from the flash area where whole_data.bin is located
 * @brief Open the flash file
 *
 * @param[in] handle ircmd's handle
 * @param[in] file_length length of the read data
 * @param[in] addr_offset start read address of the flash
 * @param[out] file_id unique identification name of the file
 *
 * @return see IrlibError_e
*/
DLLEXPORT IrlibError_e test_flash_read_file_open(IrcmdHandle_t* handle, uint32_t file_length, uint32_t addr_offset, int* file_id);


/**
 * @brief the kind of flash
 */
typedef enum {
    TEST_NAND_FLASH = 0,
    TEST_NOR_INTERNAL_FLASH = 1,
    TEST_NOR_EXTERNAL_FLASH = 2
}test_flash_type_e;


/**
 * @brief Read data from flash chosen
 *
 * @param[in] handle ircmd's handle
 * @param[in] flash_type,see test_flash_type_e
 * @param[in] offset start read address of the flash
 * @param[in] len length of the read data
 * @param[out] data the data read from flash
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e test_flash_read(IrcmdHandle_t* handle, int flash_type, int addr, int len, uint8_t* data);


/**
 * @brief Set the limit range for the output frame rate
 *
 * @param[in] handle ircmd's handle
 * @param[in] fps_limit the limit range for the output frame rate
 * 0:no limit, the frame rate when preview_start is called;others:the limit range for the output frame rate
 *
 * @return see IrlibError_e
 */
#ifndef COMMAND_IN_DEVELOPING
DLLEXPORT IrlibError_e test_frame_rate_limit_set(IrcmdHandle_t* handle, int fps_limit);
#endif

/**
 * @brief Get the limit range for the output frame rate
 *
 * @param[in] handle ircmd's handle
 * @param[out] fps_limit the limit range for the output frame rate
 * 0:no limit, the frame rate when preview_start is called;others:the limit range for the output frame rate
 *
 * @return see IrlibError_e
 */
#ifndef COMMAND_IN_DEVELOPING
DLLEXPORT IrlibError_e test_frame_rate_limit_get(IrcmdHandle_t* handle, int* fps_limit);
#endif

typedef struct
{
    uint8_t	cmd_class;
    uint8_t	mod_cmd_idx;

    uint8_t	len_l;
    uint8_t	len_h;

    uint8_t	addr_ll;
    uint8_t	addr_l;
    uint8_t	addr_h;
    uint8_t	addr_hh;

    uint16_t data_crc16;
    uint16_t header_crc16;
}vdcmd_short_header_t;

typedef struct
{
    uint8_t	cmd_class;
    uint8_t	mod_cmd_idx;
    uint8_t	sub_cmd;
    uint8_t	cmd_reserved;	//used in CFG command

    uint8_t	param_ll;
    uint8_t	param_l;
    uint8_t	param_h;
    uint8_t	param_hh;

    uint8_t	addr_ll;
    uint8_t	addr_l;
    uint8_t	addr_h;
    uint8_t	addr_hh;

    uint8_t	len_l;
    uint8_t	len_h;

    uint16_t data_crc16;
    uint16_t header_crc16;
}vdcmd_std_header_t;

typedef struct
{
    uint8_t cmd_class;
    uint8_t mod_cmd_idx;
    uint8_t sub_cmd;
    uint8_t cmd_reserved;

    uint8_t param_ll;
    uint8_t param_l;
    uint8_t param_h;
    uint8_t param_hh;

    uint8_t addr1_ll;
    uint8_t addr1_l;
    uint8_t addr1_h;
    uint8_t addr1_hh;

    uint8_t addr2_ll;
    uint8_t addr2_l;
    uint8_t addr2_h;
    uint8_t addr2_hh;

    uint8_t len_ll;
    uint8_t len_l;
    uint8_t len_h;
    uint8_t len_hh;
    uint16_t data_crc16;
    uint16_t header_crc16;
}vdcmd_long_header_t;


/**
 * @brief Send short read command
 *
 * @param ircmd_handle The handle of ircmd
 * @param vdcmd_short_header The header of short read command
 * @param length The length of data for short read command
 * @param data The data of short read command
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e short_cmd_read(IrcmdHandle_t* ircmd_handle, vdcmd_short_header_t* vdcmd_short_header, uint8_t length, uint8_t* data);

/**
 * @brief Send short write command
 *
 * @param ircmd_handle The handle of ircmd
 * @param vdcmd_short_header The header of short write command
 * @param length The length of data for short write command
 * @param data The data of short write command
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e short_cmd_write(IrcmdHandle_t* ircmd_handle, vdcmd_short_header_t* vdcmd_short_header, uint8_t length, uint8_t* data);

/**
 * @brief Send standard read command
 *
 * @param ircmd_handle The handle of ircmd
 * @param vdcmd_std_header The header of standard read command
 * @param length The length of data for standard read command
 * @param data The data of standard read command
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e standard_cmd_read(IrcmdHandle_t* ircmd_handle, vdcmd_std_header_t* vdcmd_std_header, uint32_t length, uint8_t* data);

/**
 * @brief Send standard write command
 *
 * @param ircmd_handle The handle of ircmd
 * @param vdcmd_std_header The header of standard write command
 * @param length The length of data for standard write command
 * @param data The data of standard write command
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e standard_cmd_write(IrcmdHandle_t* ircmd_handle, vdcmd_std_header_t* vdcmd_std_header, uint32_t length, uint8_t* data);

/**
 * @brief Send standard write command without read return status from device
 *
 * @param ircmd_handle The handle of ircmd
 * @param vdcmd_std_header The header of standard write command
 * @param length The length of data for standard write command
 * @param data The data of standard write command
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e standard_cmd_write_without_read_return_status(IrcmdHandle_t* ircmd_handle, vdcmd_std_header_t* vdcmd_std_header, \
    uint32_t length, uint8_t* data);

/**
 * @brief Send long read command
 *
 * @param ircmd_handle The handle of ircmd
 * @param vdcmd_long_header The header of long read command
 * @param length The length of data for long read command
 * @param data The data of long read command
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e long_cmd_read(IrcmdHandle_t* ircmd_handle, vdcmd_long_header_t* vdcmd_long_header, uint32_t length, uint8_t* data);

/**
 * @brief Send long write command
 *
 * @param ircmd_handle The handle of ircmd
 * @param vdcmd_long_header The header of long write command
 * @param length The length of data for long write command
 * @param data The data of long write command
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e long_cmd_write(IrcmdHandle_t* ircmd_handle, vdcmd_long_header_t* vdcmd_long_header, uint32_t length, uint8_t* data);

/**
 * @brief      Get cel level
 * @support    TC2-C
 *
 * @param[in]   handle ircmd's handle
 * @param[out]  level cel level, range:0-100
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_cel_level_get(IrcmdHandle_t* handle, int* level);

/**
 * @brief      Get ddr data
 * @support    TC2-C
 *
 * @param[in]   handle ircmd's handle
 * @param[in]   address
 * @param[in]   length
 * @param[out]  data
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_ddr_data_get(IrcmdHandle_t* handle, int address, int length, uint8_t* data);

/**
 * @brief      Set parameters related to multi rmcover
 *
 * @param[in]   handle ircmd's handle
 * @param[in]   index
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_multirmcover_param_set(IrcmdHandle_t* handle, int index);

/**
 * @brief the permission to switch mid stream modes
 */
typedef enum
{
    ADV_MID_PERMISSION_DISABLE = 0,
    ADV_MID_PERMISSION_ENABLE = 1
}adv_stream_mid_permission_e;

/**
 * @brief Set the permission to switch mid stream modes (only for LCE mode)
 *
 * @param[in] handle ircmd's handle
 * @param[in] permission the permission to switch mid stream modes, see adv_stream_mid_permission_e
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_stream_mid_permission_set(IrcmdHandle_t* handle, int permission);

/**
 * @brief Set homogeneity algorithm
 * Support for WN2/SE51280/SE51280T
 *
 * @param[in] handle ircmd's handle
 * @param[in] status open or close the homogeneity algorithm(0:BASIC_DISABLE  1: BASIC_ENABLE)
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_homogeneity_algorithm_set(IrcmdHandle_t* handle, int status);

/**
 * @brief Get homogeneity algorithm
 * Support for WN2/SE51280/SE51280T
 *
 * @param[in] handle ircmd's handle
 * @param[out] status open or close the homogeneity algorithm(0:BASIC_DISABLE  1: BASIC_ENABLE)
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_homogeneity_algorithm_get(IrcmdHandle_t* handle, int* status);

/**
 * @brief Set homogeneity agc and dde algorithm
 * Support for WN2/SE51280/SE51280T
 *
 * @param[in] handle ircmd's handle
 * @param[in] status open or close the homogeneity algorithm(0:BASIC_DISABLE  1: BASIC_ENABLE)
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_homogeneity_agc_dde_algorithm_set(IrcmdHandle_t* handle, int status);

/**
 * @brief Get homogeneity agc and dde algorithm
 * Support for WN2/SE51280/SE51280T
 *
 * @param[in] handle ircmd's handle
 * @param[out] status open or close the homogeneity algorithm(0:BASIC_DISABLE  1: BASIC_ENABLE)
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_homogeneity_agc_dde_algorithm_get(IrcmdHandle_t* handle, int* status);


/**
 * @brief Set environment correct switch
 * @support WN2384T, SE51280T and TC2-C
 *
 * @param[in] handle ircmd's handle
 * @param[in] status enable or disable, see basic_enable_status_e
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_env_correct_switch_set(IrcmdHandle_t* handle, int status);

/**
 * @brief Get environment correct switch
 * @support WN2384T, SE51280T and TC2-C
 *
 * @param[in] handle ircmd's handle
 * @param[in] status enable or disable, see basic_enable_status_e
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_env_correct_switch_get(IrcmdHandle_t* handle, int* status);

/**
 * @brief Set tu in environment correct
 * @support WN2384T, SE51280T and TC2-C
 *
 * @param[in] handle ircmd's handle
 * @param[in] tu reflection temperature, range:233~373 (unit: kelvin degree celsius)
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_env_correct_tu_set(IrcmdHandle_t* handle, int tu);

/**
 * @brief Get tu in environment correct
 * @support WN2384T, SE51280T and TC2-C
 *
 * @param[in] handle ircmd's handle
 * @param[in] tu reflection temperature, range:233~373 (unit: kelvin degree celsius)
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_env_correct_tu_get(IrcmdHandle_t* handle, int* tu);

/**
 * @brief Set ta in environment correct
 * @support WN2384T, SE51280T and TC2-C
 *
 * @param[in] handle ircmd's handle
 * @param[in] ta atmospheric temperature, range:233~373 (unit: kelvin degree celsius)
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_env_correct_ta_set(IrcmdHandle_t* handle, int ta);

/**
 * @brief Get ta in environment correct
 * @support WN2384T, SE51280T and TC2-C
 *
 * @param[in] handle ircmd's handle
 * @param[in] ta atmospheric temperature, range:233~373 (unit: kelvin degree celsius)
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_env_correct_ta_get(IrcmdHandle_t* handle, int* ta);

/**
 * @brief Set tau in environment correct
 * @support WN2384T, SE51280T and TC2-C
 *
 * @param[in] handle ircmd's handle
 * @param[in] tau atmospheric transmittance, range:1~16384
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_env_correct_tau_set(IrcmdHandle_t* handle, int tau);

/**
 * @brief Get tau in environment correct
 * @support WN2384T, SE51280T and TC2-C
 *
 * @param[in] handle ircmd's handle
 * @param[in] tau atmospheric transmittance, range:1~16384
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_env_correct_tau_get(IrcmdHandle_t* handle, int* tau);

/**
 * @brief Set ems in environment correct
 * @support WN2384T, SE51280T and TC2-C
 *
 * @param[in] handle ircmd's handle
 * @param[in] ems target emissivity, range:1~16384
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_env_correct_ems_set(IrcmdHandle_t* handle, int ems);

/**
 * @brief Get ems in environment correct
 * @support WN2384T, SE51280T and TC2-C
 *
 * @param[in] handle ircmd's handle
 * @param[in] ems target emissivity, range:1~16384
 *
 * @return see IrlibError_e
 */
DLLEXPORT IrlibError_e adv_env_correct_ems_get(IrcmdHandle_t* handle, int* ems);


#ifdef __cplusplus
}
#endif

#endif
