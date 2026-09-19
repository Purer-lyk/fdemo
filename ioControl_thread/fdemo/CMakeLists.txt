#set(CMAKE_C_COMPILER /usr/bin/i686-w64-mingw32-gcc)
#set(CMAKE_CXX_COMPILER /usr/bin/i686-w64-mingw32-g++)

#set(CMAKE_C_FLAGS "-m32")
#set(CMAKE_CXX_FLAGS "-m32")

cmake_minimum_required(VERSION 3.10)
add_definitions( -lwiringPi  )
project(fireDemo)

set(CMAKE_CXX_STANDARD 11)

set(MAIN_FOLDER ./master)
set(WORK_FOLDER ./process)
set(PADDLE_FOLDER ./paddle)
set(THERMAL_FOLDER ./thermal)
set(SYS_INCLUDE_DIRS /usr/local/include)
set(SYS_LIBS /usr/local/lib)
set(SYS_USB_DIRS /usr/include/libusb-1.0)
set(LOCAL_UVC_DIRS /home/l/Documents/uvc_ubuntu/libuvc/include)
set(THERMAL_INTERFACES ./thermal/interfaces)
set(THERMAL_DRIVERS ./thermal/drivers)
set(THERMAL_COMMON ./thermal/common)
set(THERMAL_OTHER ./thermal/other)
set(THERMAL_COMPONENT ./thermal/components)
set(THERMAL_THIRDPARTY ./thermal/thirdparty)
set(THERMAL_CJSON ./thermal/thirdparty/cJSON)

find_package(OpenCV 4.5 REQUIRED)
find_library(WIRINGPI_LIBRARIES NAMES wiringPi)

include_directories(${MAIN_FOLDER}
                    ${WORK_FOLDER}
                    ${OpenCV_INCLUDE_DIRS}
		    ${SYS_INCLUDE_DIRS}
		    ${PADDLE_FOLDER}
            	    ${THERMAL_FOLDER}
		    ${SYS_USB_DIRS}
		    ${LOCAL_UVC_DIRS}
		    ${THERMAL_INTERFACES}
		    ${THERMAL_DRIVERS}
		    ${THERMAL_COMMON}
		    ${THERMAL_OTHER}
		    ${THERMAL_COMPONENT}
		    ${THERMAL_THIRDPARTY}
		    ${THERMAL_CJSON})

link_directories(${PADDLE_FOLDER})
link_directories(${SYS_LIBS})
link_directories(${THERMAL_DRIVERS} ${THERMAL_THIRDPARTY})

# aux_source_directory(${WORK_FOLDER} workDir)
file(GLOB WORK_SRC ${WORK_FOLDER}/*.cpp)
file(GLOB THERMAL_SRC ${THERMAL_COMMON}/data.cpp
		      ${THERMAL_COMMON}/config.cpp 
		      ${THERMAL_COMMON}/uvc_camera.cpp
		      ${THERMAL_COMMON}/opencv_display.cpp
		      ${THERMAL_COMPONENT}/cmd.cpp
		      ${THERMAL_COMPONENT}/temp_measure.cpp
 		      ${THERMAL_CJSON}/cJSON.c
)

add_executable(fireDemo ${MAIN_FOLDER}/main.cpp ${MAIN_FOLDER}/sample.cpp ${WORK_SRC} ${THERMAL_SRC})

#add_library(workSrc ${workDir})

#target_link_libraries(fireDemo workSrc)
target_link_libraries(fireDemo libmodbus.so)
target_link_libraries(fireDemo ${OpenCV_LIBS})
target_link_libraries(fireDemo ${WIRINGPI_LIBRARIES})
target_link_libraries(fireDemo ircmd iruart iruvc ircam irtemp irparse pthread usb-1.0 -lm)
target_link_libraries(fireDemo libpaddle_light_api_shared.so)

#message(${OpenCV_INCLUDE_DIRS})


