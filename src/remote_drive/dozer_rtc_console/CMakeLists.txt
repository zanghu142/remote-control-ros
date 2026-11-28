cmake_minimum_required(VERSION 3.18)
project(dozer_rtc_console LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)
if(WIN32)
  include(cmake/windows.cmake)
else()
  find_package(ament_cmake REQUIRED)
  find_package(dozer_audio REQUIRED)
  find_package(dozer_media_vendor REQUIRED)
  find_package(dozer_rtc_sdk REQUIRED)
  find_package(dozer_rtc_common REQUIRED)
  find_package(imgui CONFIG REQUIRED)
endif()

set(app "${CMAKE_CURRENT_SOURCE_DIR}/src/app")
add_library(console_options STATIC "${app}/rtc_console_options.cpp")
target_include_directories(console_options PUBLIC "${app}")
target_link_libraries(console_options PUBLIC dozer_rtc_common::dozer_rtc_common)

add_library(console_support STATIC
  "${app}/stereo_fisheye.cpp"
  "${app}/video_composite_layout.cpp"
  "${app}/video_detection_overlay.cpp"
  "${app}/video_view_state.cpp"
  "${app}/vehicle_control_sender.cpp"
  "${app}/vehicle_state_log_limiter.cpp")
target_link_libraries(console_support PUBLIC console_options Threads::Threads)

add_executable(rtc_console "${app}/main.cpp")
target_link_libraries(rtc_console PRIVATE console_support dozer_audio::dozer_audio
  dozer_rtc_sdk::vtsrtc imgui::imgui OpenGL::GL ${CMAKE_DL_LIBS})
target_compile_definitions(rtc_console PRIVATE DOZER_RTC_HAS_VIDEO_DELAY_STATS=1)
if(WIN32)
  target_sources(rtc_console PRIVATE "${app}/stereo_fisheye_renderer.cpp")
  target_link_libraries(rtc_console PRIVATE dwmapi)
  target_sources(rtc_console PRIVATE "${app}/rtc_console.manifest")
  set(console_install_dir bin)
  install(TARGETS rtc_console vtsrtc RUNTIME DESTINATION bin COMPONENT Runtime)
else()
  include(../../../cmake/rtc_runtime.cmake)
  dozer_install_rtc_app(rtc_console)
  set(console_install_dir lib/${PROJECT_NAME})
endif()

set(rtc_config_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../system/dozer_bringup/config/rtc_console")
foreach(config_name rtc.cfg console.cfg)
  configure_file("${rtc_config_dir}/${config_name}" "${CMAKE_CURRENT_BINARY_DIR}/${config_name}" COPYONLY)
  if(WIN32)
    file(GENERATE OUTPUT "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/$<CONFIG>/${config_name}"
      INPUT "${CMAKE_CURRENT_BINARY_DIR}/${config_name}")
  endif()
endforeach()
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/rtc.cfg" "${CMAKE_CURRENT_BINARY_DIR}/console.cfg"
  DESTINATION ${console_install_dir} COMPONENT Runtime)

if(WIN32)
  set(fisheye_calibrations
    "${rtc_config_dir}/../stereo_fisheye_extrinsics.yml"
    "${rtc_config_dir}/../surround_front_fisheye.yml"
    "${rtc_config_dir}/../surround_rear_fisheye.yml"
    "${rtc_config_dir}/../surround_left_fisheye.yml"
    "${rtc_config_dir}/../surround_right_fisheye.yml")
  set(fisheye_calibration_output "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/$<CONFIG>/fisheye_calibration")
  # Run on incremental builds too: updated or deleted calibration copies must
  # be refreshed even when rtc_console.exe itself does not need relinking.
  add_custom_target(rtc_console_calibration
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${fisheye_calibration_output}"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${fisheye_calibrations}
      "${fisheye_calibration_output}"
    VERBATIM)
  add_dependencies(rtc_console rtc_console_calibration)
  install(FILES ${fisheye_calibrations}
    DESTINATION "${console_install_dir}/fisheye_calibration" COMPONENT Runtime)
endif()

if(BUILD_TESTING)
  enable_testing()
  function(console_test name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE console_support)
    add_test(NAME ${name} COMMAND ${name})
    set_tests_properties(${name} PROPERTIES TIMEOUT 30)
  endfunction()
  console_test(rtc_console_options_tests test/rtc_console_options_tests.cpp)
  console_test(rtc_console_fused_detection_tests test/fused_detection_tests.cpp)
  console_test(rtc_console_vehicle_control_sender_tests test/vehicle_control_sender_tests.cpp)
  console_test(rtc_console_vehicle_state_log_tests test/vehicle_state_log_limiter_tests.cpp)
  console_test(rtc_console_video_view_state_tests test/video_view_state_tests.cpp)
  console_test(rtc_console_video_composite_layout_tests test/video_composite_layout_tests.cpp)
  console_test(rtc_console_video_detection_overlay_tests test/video_detection_overlay_tests.cpp)
  console_test(rtc_console_video_fullscreen_state_tests test/video_fullscreen_state_tests.cpp)
  add_test(NAME rtc_console_config COMMAND rtc_console_options_tests
    "${CMAKE_CURRENT_BINARY_DIR}/rtc.cfg")
  set_tests_properties(rtc_console_config PROPERTIES TIMEOUT 30)
  add_test(NAME rtc_console_help COMMAND rtc_console --help)
  set_tests_properties(rtc_console_help PROPERTIES TIMEOUT 10)
  if(WIN32)
    add_test(NAME rtc_console_windows_install COMMAND "${CMAKE_COMMAND}"
      "-DBUILD_DIR=${CMAKE_BINARY_DIR}" "-DCONFIG=$<CONFIG>"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/test/windows_install_test.cmake")
    set_tests_properties(rtc_console_windows_install PROPERTIES TIMEOUT 30)
  elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    find_program(CONSOLE_READELF readelf REQUIRED)
    find_program(CONSOLE_LDD ldd REQUIRED)
    add_test(NAME rtc_console_sdk_linkage COMMAND "${CMAKE_COMMAND}"
      "-DCONSOLE_EXECUTABLE=$<TARGET_FILE:rtc_console>"
      "-DSDK_LIBRARY=$<TARGET_FILE:dozer_rtc_sdk::vtsrtc>"
      "-DREADELF_TOOL=${CONSOLE_READELF}"
      "-DLDD_TOOL=${CONSOLE_LDD}"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/test/sdk_linkage_test.cmake")
    set_tests_properties(rtc_console_sdk_linkage PROPERTIES TIMEOUT 10)
  endif()
endif()

if(NOT WIN32)
  ament_package()
endif()
