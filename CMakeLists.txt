cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM3)

# vcpkg 설정
set(CMAKE_TOOLCHAIN_FILE "/opt/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

# Add visualization option
option(BUILD_WITH_VISUALIZATION "Build with visualization support" OFF)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -w -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.4)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 CONFIG REQUIRED)

# Find dependencies via vcpkg
find_package(Sophus CONFIG REQUIRED)
find_package(g2o CONFIG REQUIRED)

# Conditionally find Pangolin
if(BUILD_WITH_VISUALIZATION)
    find_package(Pangolin REQUIRED)
    add_definitions(-DUSE_VISUALIZATION)
endif()

find_package(realsense2)

add_subdirectory(${PROJECT_SOURCE_DIR}/external/DBoW2)
add_subdirectory(${PROJECT_SOURCE_DIR}/external/DLib)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/CameraModels
${EIGEN3_INCLUDE_DIR}
${PROJECT_SOURCE_DIR}/external/DBoW2
${PROJECT_SOURCE_DIR}/external/DLibs
${g2o_INCLUDE_DIRS}
${Sophus_INCLUDE_DIRS}
)

if(BUILD_WITH_VISUALIZATION)
    include_directories(${Pangolin_INCLUDE_DIRS})
endif()

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# Define source files
set(ORB_SLAM3_SOURCES
	src/System.cc
	src/Tracking.cc
	src/LocalMapping.cc
	src/LoopClosing.cc
	src/ORBextractor.cc
	src/ORBmatcher.cc
	src/Converter.cc
	src/MapPoint.cc
	src/KeyFrame.cc
	src/Atlas.cc
	src/Map.cc
	src/Optimizer.cc
	src/Frame.cc
	src/KeyFrameDatabase.cc
	src/Sim3Solver.cc
	src/ImuTypes.cc
	src/G2oTypes.cc
	src/CameraModels/Pinhole.cpp
	src/CameraModels/KannalaBrandt8.cpp
	src/OptimizableTypes.cpp
	src/MLPnPsolver.cpp
	src/GeometricTools.cc
	src/TwoViewReconstruction.cc
	src/Config.cc
	src/Settings.cc
)

# Add visualization sources conditionally
if(BUILD_WITH_VISUALIZATION)
    list(APPEND ORB_SLAM3_SOURCES
        src/FrameDrawer.cc
        src/MapDrawer.cc
        src/Viewer.cc
    )
endif()

add_library(${PROJECT_NAME} SHARED ${ORB_SLAM3_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/../DBoW2>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/../DLibs>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
	DBoW2
	DLib
	g2o::core
	g2o::types_sba
	g2o::types_slam3d
	g2o::types_slam3d_addons
	g2o::types_slam2d
	g2o::types_slam2d_addons
	g2o::types_sim3
	g2o::solver_dense
	g2o::solver_eigen
	Sophus::Sophus
	Eigen3::Eigen
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  -lboost_serialization
)

if(BUILD_WITH_VISUALIZATION)
    target_link_libraries(${PROJECT_NAME} PUBLIC ${Pangolin_LIBRARIES})
endif()

# If RealSense SDK is found the library is added and its examples compiled
if(realsense2_FOUND)
    include_directories(${PROJECT_NAME}
    ${realsense_INCLUDE_DIR}
    )
    target_link_libraries(${PROJECT_NAME}
    ${realsense2_LIBRARY}
    )
endif()

# Build examples only if visualization is enabled
if(BUILD_WITH_VISUALIZATION)
    # RGB-D examples
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

    add_executable(rgbd_tum
            Examples/RGB-D/rgbd_tum.cc)
    target_link_libraries(rgbd_tum ${PROJECT_NAME})

    if(realsense2_FOUND)
        add_executable(rgbd_realsense_D435i
                Examples/RGB-D/rgbd_realsense_D435i.cc)
        target_link_libraries(rgbd_realsense_D435i ${PROJECT_NAME})
    endif()

    # RGB-D inertial examples
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D-Inertial)

    if(realsense2_FOUND)
        add_executable(rgbd_inertial_realsense_D435i
                Examples/RGB-D-Inertial/rgbd_inertial_realsense_D435i.cc)
        target_link_libraries(rgbd_inertial_realsense_D435i ${PROJECT_NAME})
    endif()

    #Stereo examples
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

    add_executable(stereo_kitti
            Examples/Stereo/stereo_kitti.cc)
    target_link_libraries(stereo_kitti ${PROJECT_NAME})

    add_executable(stereo_euroc
            Examples/Stereo/stereo_euroc.cc)
    target_link_libraries(stereo_euroc ${PROJECT_NAME})

    add_executable(stereo_tum_vi
            Examples/Stereo/stereo_tum_vi.cc)
    target_link_libraries(stereo_tum_vi ${PROJECT_NAME})

    if(realsense2_FOUND)
        add_executable(stereo_realsense_t265
                Examples/Stereo/stereo_realsense_t265.cc)
        target_link_libraries(stereo_realsense_t265 ${PROJECT_NAME})

        add_executable(stereo_realsense_D435i
                Examples/Stereo/stereo_realsense_D435i.cc)
        target_link_libraries(stereo_realsense_D435i ${PROJECT_NAME})
    endif()

    #Monocular examples
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

    add_executable(mono_tum
            Examples/Monocular/mono_tum.cc)
    target_link_libraries(mono_tum ${PROJECT_NAME})

    add_executable(mono_kitti
            Examples/Monocular/mono_kitti.cc)
    target_link_libraries(mono_kitti ${PROJECT_NAME})

    add_executable(mono_euroc
            Examples/Monocular/mono_euroc.cc)
    target_link_libraries(mono_euroc ${PROJECT_NAME})

    add_executable(mono_tum_vi
            Examples/Monocular/mono_tum_vi.cc)
    target_link_libraries(mono_tum_vi ${PROJECT_NAME})

    if(realsense2_FOUND)
        add_executable(mono_realsense_t265
                Examples/Monocular/mono_realsense_t265.cc)
        target_link_libraries(mono_realsense_t265 ${PROJECT_NAME})

        add_executable(mono_realsense_D435i
                Examples/Monocular/mono_realsense_D435i.cc)
        target_link_libraries(mono_realsense_D435i ${PROJECT_NAME})
    endif()

    #Monocular inertial examples
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular-Inertial)

    add_executable(mono_inertial_euroc
            Examples/Monocular-Inertial/mono_inertial_euroc.cc)
    target_link_libraries(mono_inertial_euroc ${PROJECT_NAME})

    add_executable(mono_inertial_tum_vi
            Examples/Monocular-Inertial/mono_inertial_tum_vi.cc)
    target_link_libraries(mono_inertial_tum_vi ${PROJECT_NAME})

    if(realsense2_FOUND)
        add_executable(mono_inertial_realsense_t265
                Examples/Monocular-Inertial/mono_inertial_realsense_t265.cc)
        target_link_libraries(mono_inertial_realsense_t265 ${PROJECT_NAME})

        add_executable(mono_inertial_realsense_D435i
                Examples/Monocular-Inertial/mono_inertial_realsense_D435i.cc)
        target_link_libraries(mono_inertial_realsense_D435i ${PROJECT_NAME})
    endif()

    #Stereo Inertial examples
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo-Inertial)

    add_executable(stereo_inertial_euroc
            Examples/Stereo-Inertial/stereo_inertial_euroc.cc)
    target_link_libraries(stereo_inertial_euroc ${PROJECT_NAME})

    add_executable(stereo_inertial_tum_vi
            Examples/Stereo-Inertial/stereo_inertial_tum_vi.cc)
    target_link_libraries(stereo_inertial_tum_vi ${PROJECT_NAME})

    if(realsense2_FOUND)
        add_executable(stereo_inertial_realsense_t265
                Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc)
        target_link_libraries(stereo_inertial_realsense_t265 ${PROJECT_NAME})

        add_executable(stereo_inertial_realsense_D435i
                Examples/Stereo-Inertial/stereo_inertial_realsense_D435i.cc)
        target_link_libraries(stereo_inertial_realsense_D435i ${PROJECT_NAME})
    endif()
endif()
