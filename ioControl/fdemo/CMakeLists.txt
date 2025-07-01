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
set(MODBUS_INCLUDE_DIRS /usr/local/include)
set(MODBUS_LIBS /usr/local/lib)

find_package(OpenCV 4.5 REQUIRED)
find_library(WIRINGPI_LIBRARIES NAMES wiringPi)

include_directories(${MAIN_FOLDER}
                    ${WORK_FOLDER}
                    ${OpenCV_INCLUDE_DIRS}
		    ${MODBUS_INCLUDE_DIRS}
		    ${PADDLE_FOLDER})

link_directories(${PADDLE_FOLDER})
link_directories(${MODBUS_LIBS})

aux_source_directory(${WORK_FOLDER} workDir)

add_executable(fireDemo ${MAIN_FOLDER}/main.cpp)

add_library(workSrc ${workDir})

target_link_libraries(fireDemo PRIVATE workSrc)
target_link_libraries(fireDemo PRIVATE libmodbus.so)
target_link_libraries(fireDemo PRIVATE ${OpenCV_LIBS})
target_link_libraries(fireDemo PRIVATE ${WIRINGPI_LIBRARIES}  -lpthread)
target_link_libraries(fireDemo PRIVATE libpaddle_light_api_shared.so)

message(${OpenCV_INCLUDE_DIRS})


