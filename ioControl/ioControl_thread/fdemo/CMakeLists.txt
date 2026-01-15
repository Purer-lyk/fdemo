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

find_package(OpenCV 4.5 REQUIRED)
find_library(WIRINGPI_LIBRARIES NAMES wiringPi)

include_directories(${MAIN_FOLDER}
                    ${WORK_FOLDER}
                    ${OpenCV_INCLUDE_DIRS}
		    ${SYS_INCLUDE_DIRS}
		    ${PADDLE_FOLDER}
            	    ${THERMAL_FOLDER}
		    ${SYS_USB_DIRS}
		    ${LOCAL_UVC_DIRS})

link_directories(${PADDLE_FOLDER})
link_directories(${SYS_LIBS})

# aux_source_directory(${WORK_FOLDER} workDir)
file(GLOB WORK_SRC ${WORK_FOLDER}/*.cpp)
file(GLOB THERMAL_SRC ${THERMAL_FOLDER}/*.c ${THERMAL_FOLDER}/*.cpp)

add_executable(fireDemo ${MAIN_FOLDER}/main.cpp ${WORK_SRC} ${THERMAL_SRC})

# add_library(workSrc ${workDir})

# target_link_libraries(fireDemo PRIVATE workSrc)
target_link_libraries(fireDemo PRIVATE libmodbus.so)
target_link_libraries(fireDemo PRIVATE ${OpenCV_LIBS})
target_link_libraries(fireDemo PRIVATE ${WIRINGPI_LIBRARIES} -lusb-1.0 -luvc -lpthread)
target_link_libraries(fireDemo PRIVATE libpaddle_light_api_shared.so)

message(${OpenCV_INCLUDE_DIRS})


