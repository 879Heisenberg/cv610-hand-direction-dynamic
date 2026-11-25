cmake_minimum_required(VERSION 3.16)

project(hand_direction VERSION 0.1.0 LANGUAGES C)

# Embedded inference is CPU-bound during preprocessing. Default to an optimized
# build unless the caller explicitly selected another build type.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(HAND_DIRECTION_BUILD_TESTS "Build host-side unit tests" ON)
option(HAND_DIRECTION_ENABLE_CV610 "Build the Hi3516CV610 ACL detector" OFF)
set(HAND_DIRECTION_SDK_ROOT "" CACHE PATH "Hi3516CV610 SOC SDK root")
set(HAND_DIRECTION_CV610_SENSOR_TYPE
    "OS04D10_MIPI_4M_30FPS_10BIT"
    CACHE STRING "CV610 sensor configuration used by sample_comm")
set(HAND_DIRECTION_CV610_SAMPLE_COMM_DIR
    "/root/work/hi_aiot_solution/hi_aiot_solution/apps/cv610_ev_board/samples/capture_picture/comm"
    CACHE PATH "CV610 capture_picture sample common source directory")
set(HAND_DIRECTION_DRM_ROOT
    "/root/work/hi_aiot_solution/hi_aiot_solution/_build/open_source/drm/release"
    CACHE PATH "Cross-compiled libdrm prefix for the CV610 display output")

set(HAND_DIRECTION_CORE_SOURCES
    src/detector.c
    src/detector_stub.c
    src/preprocess.c
    src/postprocess.c
    src/tracker.c
    src/trajectory.c
    src/direction.c
    src/gesture_engine.c
    src/output_console.c
)

if(HAND_DIRECTION_ENABLE_CV610)
    if(HAND_DIRECTION_SDK_ROOT STREQUAL "")
        message(FATAL_ERROR
            "HAND_DIRECTION_SDK_ROOT is required when HAND_DIRECTION_ENABLE_CV610=ON")
    endif()
    set(HAND_DIRECTION_SDK_INCLUDE_DIR "${HAND_DIRECTION_SDK_ROOT}/include")
    set(HAND_DIRECTION_SDK_LIBRARY_DIR "${HAND_DIRECTION_SDK_ROOT}/libs/musl")
    set(HAND_DIRECTION_SDK_CONFIG_INCLUDE_DIR "${HAND_DIRECTION_SDK_ROOT}/config/gcc")
    get_filename_component(HAND_DIRECTION_CV610_PLATFORM_ROOT
        "${HAND_DIRECTION_SDK_ROOT}/.." ABSOLUTE)
    if(NOT EXISTS "${HAND_DIRECTION_SDK_INCLUDE_DIR}/svp_acl.h")
        message(FATAL_ERROR
            "svp_acl.h was not found under ${HAND_DIRECTION_SDK_INCLUDE_DIR}")
    endif()
    if(NOT EXISTS "${HAND_DIRECTION_SDK_LIBRARY_DIR}")
        message(FATAL_ERROR
            "CV610 musl library directory was not found: ${HAND_DIRECTION_SDK_LIBRARY_DIR}")
    endif()
    if(NOT EXISTS "${HAND_DIRECTION_CV610_SAMPLE_COMM_DIR}/sample_comm.h")
        message(FATAL_ERROR
            "CV610 sample_comm.h was not found under "
            "${HAND_DIRECTION_CV610_SAMPLE_COMM_DIR}")
    endif()
    if(NOT EXISTS "${HAND_DIRECTION_DRM_ROOT}/include/xf86drm.h" OR
       NOT EXISTS "${HAND_DIRECTION_DRM_ROOT}/include/xf86drmMode.h" OR
       NOT EXISTS "${HAND_DIRECTION_DRM_ROOT}/lib/libdrm.a")
        message(FATAL_ERROR
            "Cross-compiled libdrm was not found under ${HAND_DIRECTION_DRM_ROOT}")
    endif()

    set(HAND_DIRECTION_CV610_SAMPLE_COMMON_SOURCES
        "${HAND_DIRECTION_CV610_PLATFORM_ROOT}/samples/mpp/common/sample_comm_audio.c"
        "${HAND_DIRECTION_CV610_SAMPLE_COMM_DIR}/sample_comm_sys.c"
        "${HAND_DIRECTION_CV610_SAMPLE_COMM_DIR}/sample_comm_vi.c"
        "${HAND_DIRECTION_CV610_SAMPLE_COMM_DIR}/sample_comm_vpss.c"
        "${HAND_DIRECTION_CV610_SAMPLE_COMM_DIR}/sample_comm_isp.c"
        "${HAND_DIRECTION_CV610_SAMPLE_COMM_DIR}/sample_comm_venc.c"
        "${HAND_DIRECTION_CV610_SAMPLE_COMM_DIR}/loadbmp.c"
    )
    foreach(HAND_DIRECTION_CV610_SAMPLE_COMMON_SOURCE
            IN LISTS HAND_DIRECTION_CV610_SAMPLE_COMMON_SOURCES)
        if(NOT EXISTS "${HAND_DIRECTION_CV610_SAMPLE_COMMON_SOURCE}")
            message(FATAL_ERROR
                "Required CV610 sample common source was not found: "
                "${HAND_DIRECTION_CV610_SAMPLE_COMMON_SOURCE}")
        endif()
    endforeach()

    add_library(hand_direction_cv610_sample_comm STATIC
        ${HAND_DIRECTION_CV610_SAMPLE_COMMON_SOURCES})
    target_include_directories(hand_direction_cv610_sample_comm
        PRIVATE
            ${HAND_DIRECTION_SDK_INCLUDE_DIR}
            ${HAND_DIRECTION_SDK_INCLUDE_DIR}/module
            ${HAND_DIRECTION_SDK_CONFIG_INCLUDE_DIR}
            ${HAND_DIRECTION_CV610_SAMPLE_COMM_DIR}
    )
    target_compile_definitions(hand_direction_cv610_sample_comm
        PRIVATE
            SENSOR0_TYPE=${HAND_DIRECTION_CV610_SENSOR_TYPE}
            ${HAND_DIRECTION_CV610_SENSOR_TYPE}_SELECT=y
            OT_ACODEC_TYPE_INNER
            _GNU_SOURCE
    )
    target_compile_options(hand_direction_cv610_sample_comm
        PRIVATE
            -Wall -Wno-unused-variable -Wno-unused-parameter -Wno-unused-function
    )
else()
    list(APPEND HAND_DIRECTION_CORE_SOURCES src/detector_cv610_placeholder.c)
endif()

add_library(hand_direction_core STATIC ${HAND_DIRECTION_CORE_SOURCES})

target_include_directories(hand_direction_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_features(hand_direction_core PUBLIC c_std_11)

if(MSVC)
    target_compile_options(hand_direction_core PRIVATE /W4)
else()
    target_compile_options(hand_direction_core PRIVATE -Wall -Wextra -Wpedantic -O3)
    if(UNIX)
        target_link_libraries(hand_direction_core PUBLIC m)
    endif()
endif()

if(HAND_DIRECTION_ENABLE_CV610)
    set(HAND_DIRECTION_CV610_STATIC_LIBRARIES
        "${HAND_DIRECTION_SDK_LIBRARY_DIR}/libot_base.a"
        "${HAND_DIRECTION_SDK_LIBRARY_DIR}/libot_mpi_sysmem.a"
        "${HAND_DIRECTION_SDK_LIBRARY_DIR}/libot_irq.a"
        "${HAND_DIRECTION_SDK_LIBRARY_DIR}/libot_sys.a"
        "${HAND_DIRECTION_SDK_LIBRARY_DIR}/libsecurec.a"
    )
    foreach(HAND_DIRECTION_SDK_LIBRARY IN LISTS HAND_DIRECTION_CV610_STATIC_LIBRARIES)
        if(NOT EXISTS "${HAND_DIRECTION_SDK_LIBRARY}")
            message(FATAL_ERROR
                "Required CV610 static library was not found: ${HAND_DIRECTION_SDK_LIBRARY}")
        endif()
    endforeach()

    add_library(hand_direction_cv610 STATIC src/detector_cv610.c)
    target_include_directories(hand_direction_cv610
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src
            ${HAND_DIRECTION_SDK_INCLUDE_DIR}
    )
    target_compile_features(hand_direction_cv610 PUBLIC c_std_11)
    target_link_libraries(hand_direction_cv610 PUBLIC hand_direction_core)

    if(MSVC)
        target_compile_options(hand_direction_cv610 PRIVATE /W4)
    else()
        target_compile_options(hand_direction_cv610 PRIVATE -Wall -Wextra -Wpedantic -O3)
        # libsvp_acl and libot_osal have circular references into the SDK's
        # static MPP/base/securec archives. Keep the whole dependency set in
        # one group and reject shared-library symbols that are not resolved.
        target_link_libraries(hand_direction_cv610 PRIVATE
            -Wl,--start-group
                ${HAND_DIRECTION_CV610_STATIC_LIBRARIES}
                -Wl,--no-as-needed
                "${HAND_DIRECTION_SDK_LIBRARY_DIR}/libot_osal.so"
                "${HAND_DIRECTION_SDK_LIBRARY_DIR}/libsvp_acl.so"
                "${HAND_DIRECTION_SDK_LIBRARY_DIR}/libprotobuf-c.so"
                -Wl,--as-needed
                stdc++
                pthread
                m
                dl
                c
            -Wl,--end-group
            -Wl,--no-allow-shlib-undefined
        )
    endif()
endif()

add_executable(hand_direction_demo app/main.c)
target_link_libraries(hand_direction_demo PRIVATE hand_direction_core)

add_executable(hand_direction_bench app/replay_bench.c)
target_link_libraries(hand_direction_bench PRIVATE hand_direction_core)
if(MSVC)
    target_compile_options(hand_direction_bench PRIVATE /W4)
else()
    target_compile_options(hand_direction_bench PRIVATE -Wall -Wextra -Wpedantic -O3)
endif()

if(HAND_DIRECTION_ENABLE_CV610)
    add_executable(cv610_smoke app/smoke.c)
    target_link_libraries(cv610_smoke PRIVATE hand_direction_cv610)
    set_target_properties(cv610_smoke PROPERTIES
        BUILD_RPATH "\$ORIGIN/lib"
        INSTALL_RPATH "\$ORIGIN/lib"
        BUILD_WITH_INSTALL_RPATH TRUE
        INSTALL_RPATH_USE_LINK_PATH FALSE
    )

    add_executable(cv610_live_hand
        app/live_main.c
        app/live_media.c
        app/display_drm.c
        app/audio_announcer.c
    )
    target_include_directories(cv610_live_hand
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}/app
            ${HAND_DIRECTION_SDK_INCLUDE_DIR}
            ${HAND_DIRECTION_SDK_INCLUDE_DIR}/module
            ${HAND_DIRECTION_SDK_CONFIG_INCLUDE_DIR}
            ${HAND_DIRECTION_CV610_SAMPLE_COMM_DIR}
            ${HAND_DIRECTION_DRM_ROOT}/include
            ${HAND_DIRECTION_DRM_ROOT}/include/libdrm
    )
    target_compile_definitions(cv610_live_hand
        PRIVATE
            SENSOR0_TYPE=${HAND_DIRECTION_CV610_SENSOR_TYPE}
            ${HAND_DIRECTION_CV610_SENSOR_TYPE}_SELECT=y
            OT_ACODEC_TYPE_INNER
            _GNU_SOURCE
    )
    target_link_directories(cv610_live_hand
        PRIVATE "${HAND_DIRECTION_SDK_LIBRARY_DIR}")
    target_link_libraries(cv610_live_hand
        PRIVATE
            hand_direction_cv610
            hand_direction_cv610_sample_comm
            -Wl,--start-group
                ss_mpi ss_mpi_sysbind ss_mpi_sysmem
                ss_mpi_ae ss_mpi_isp ot_mpi_isp ss_mpi_awb
                dehaze extend_stats drc ldci bnr calcflicker ir_auto acs
                sns_os04d10
                ot_osal ot_base ot_sys ot_vb ot_pm ot_vca ot_irq
                ot_sensor_i2c
                ot_vi ot_vpss ot_vpp ot_vgs
                ot_chnl ot_rgn
                ot_isp ot_aiisp
                ot_venc ot_rc ot_h264e ot_h265e ot_svac3e ot_jpege
                ss_mpi_audio ss_mpi_audio_adp
                voice_engine upvqe dnvqe vqe_common
                opus aac_dec aac_enc aac_sbr_dec aac_sbr_enc aac_comm
                vqe_res vqe_record vqe_hpf vqe_anr vqe_aec vqe_agc vqe_eq vqe_talkv2
                mp3_enc mp3_dec mp3_lame
                securec
                ${HAND_DIRECTION_DRM_ROOT}/lib/libdrm.a
                pthread dl m
            -Wl,--end-group
    )
    set_target_properties(cv610_live_hand PROPERTIES
        BUILD_RPATH "\$ORIGIN/lib"
        INSTALL_RPATH "\$ORIGIN/lib"
        BUILD_WITH_INSTALL_RPATH TRUE
        INSTALL_RPATH_USE_LINK_PATH FALSE
    )

    add_executable(cv610_gesture_sound app/audio_announcer.c)
    target_include_directories(cv610_gesture_sound
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}/app
            ${HAND_DIRECTION_SDK_INCLUDE_DIR}
            ${HAND_DIRECTION_SDK_INCLUDE_DIR}/module
            ${HAND_DIRECTION_SDK_CONFIG_INCLUDE_DIR}
            ${HAND_DIRECTION_CV610_SAMPLE_COMM_DIR}
    )
    target_compile_definitions(cv610_gesture_sound
        PRIVATE
            HAND_GESTURE_SOUND_PLAYER=1
            SENSOR0_TYPE=${HAND_DIRECTION_CV610_SENSOR_TYPE}
            ${HAND_DIRECTION_CV610_SENSOR_TYPE}_SELECT=y
            OT_ACODEC_TYPE_INNER
            _GNU_SOURCE
    )
    target_link_directories(cv610_gesture_sound
        PRIVATE "${HAND_DIRECTION_SDK_LIBRARY_DIR}")
    target_link_libraries(cv610_gesture_sound
        PRIVATE
            hand_direction_cv610_sample_comm
            -Wl,--start-group
                ss_mpi ss_mpi_sysbind ss_mpi_sysmem
                ot_osal ot_base ot_sys ot_vb ot_pm ot_vca ot_irq
                ot_sensor_i2c ot_chnl ot_rgn
                ss_mpi_audio ss_mpi_audio_adp
                voice_engine upvqe dnvqe vqe_common
                opus aac_dec aac_enc aac_sbr_dec aac_sbr_enc aac_comm
                vqe_res vqe_record vqe_hpf vqe_anr vqe_aec vqe_agc vqe_eq vqe_talkv2
                mp3_enc mp3_dec mp3_lame
                securec
                pthread dl m
            -Wl,--end-group
    )
    set_target_properties(cv610_gesture_sound PROPERTIES
        BUILD_RPATH "\$ORIGIN/lib"
        INSTALL_RPATH "\$ORIGIN/lib"
        BUILD_WITH_INSTALL_RPATH TRUE
        INSTALL_RPATH_USE_LINK_PATH FALSE
    )
endif()

if(HAND_DIRECTION_BUILD_TESTS)
    enable_testing()

    add_executable(test_direction tests/test_direction.c)
    target_link_libraries(test_direction PRIVATE hand_direction_core)
    add_test(NAME direction COMMAND test_direction)

    add_executable(test_gesture_engine tests/test_gesture_engine.c)
    target_link_libraries(test_gesture_engine PRIVATE hand_direction_core)
    add_test(NAME gesture_engine COMMAND test_gesture_engine)

    add_executable(test_preprocess tests/test_preprocess.c)
    target_link_libraries(test_preprocess PRIVATE hand_direction_core)
    add_test(NAME preprocess COMMAND test_preprocess)

    add_executable(test_postprocess tests/test_postprocess.c)
    target_link_libraries(test_postprocess PRIVATE hand_direction_core)
    add_test(NAME postprocess COMMAND test_postprocess)

    add_test(NAME replay_bench COMMAND hand_direction_bench --self-test)
endif()
