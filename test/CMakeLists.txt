# PROJECT(test)
 
#cmake_minimum_required(VERSION 3.1.0)

find_package(OpenCV REQUIRED)

find_package(box2d REQUIRED)

find_package(PkgConfig)

find_package(Threads)

include(CTest)

#pkg_check_modules(clock REQUIRED IMPORTED_TARGET libCLoCK)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64") # for Raspberry Pi
	find_package(iir)
	pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)
	include_directories(${LIBCAMERA_INCLUDE_DIRS})
	add_executable(testController iir_test_headers.h testController.cpp)
	add_executable(task_ofs task_ofs.cpp)
	add_executable(task_avgofs task_avgofs.cpp)
	add_executable(test_IIR iir_test_headers.h test_IIR.cpp)
	target_link_libraries(testController box2d iir::iir_static PkgConfig::LIBCAMERA cam2opencv ${OpenCV_LIBS} CloCK alphabot pthread pigpio)
	target_link_libraries(task_ofs box2d PkgConfig::LIBCAMERA cam2opencv ${OpenCV_LIBS} CloCK alphabot pthread pigpio)
	target_link_libraries(task_avgofs box2d PkgConfig::LIBCAMERA cam2opencv ${OpenCV_LIBS} CloCK alphabot pthread pigpio)
	target_link_libraries(test_IIR box2d iir::iir_static PkgConfig::LIBCAMERA cam2opencv ${OpenCV_LIBS} CloCK alphabot pthread pigpio)
	message("build test cases")
else()
	message("making simulation test cases")
	add_executable(targetless_sim callbacks.h test_essentials.h targetless_sim.cpp)
	add_executable(target_sim callbacks.h test_essentials.h target_sim.cpp)
	add_executable(loop_RT callbacks.h test_essentials.h loop_RT.cpp)
	add_executable(loop_sim callbacks.h test_essentials.h loop_sim.cpp)
	add_executable(testLoop callbacks.h test_essentials.h testLoop.cpp)
	add_executable(update_graph callbacks.h test_essentials.h update_graph.cpp)
	target_link_libraries(target_sim CloCK ${OpenCV_LIBS} box2d rt pthread cpptimer)
	#target_link_libraries(target_reactive_sim CloCK ${OpenCV_LIBS} box2d rt pthread cpptimer)
	target_link_libraries(targetless_sim CloCK ${OpenCV_LIBS} box2d rt pthread cpptimer)
	target_link_libraries(loop_RT CloCK ${OpenCV_LIBS} box2d rt pthread cpptimer)
	target_link_libraries(loop_sim CloCK ${OpenCV_LIBS} box2d rt pthread cpptimer)
	target_link_libraries(testLoop CloCK ${OpenCV_LIBS} box2d rt pthread cpptimer)
	target_link_libraries(update_graph CloCK ${OpenCV_LIBS} box2d rt pthread cpptimer)			
endif()

set(CMAKE_BUILD_TYPE Debug)

add_compile_options(-Wall -pedantic)
