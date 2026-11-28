set(prefix "${BUILD_DIR}/install-test/${CONFIG}")
execute_process(COMMAND "${CMAKE_COMMAND}" --install "${BUILD_DIR}" --config "${CONFIG}"
  --prefix "${prefix}" --component Runtime
  RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "Install failed: ${output}${error}")
endif()
foreach(name rtc_console.exe vtsrtc.dll rtc.cfg console.cfg
    fisheye_calibration/stereo_fisheye_extrinsics.yml)
  if(NOT EXISTS "${prefix}/bin/${name}")
    message(FATAL_ERROR "Missing installed runtime file: ${name}")
  endif()
endforeach()
# 从安装目录之外运行，且仅保留系统 PATH，检查交付物是否自包含。
execute_process(COMMAND "${CMAKE_COMMAND}" -E env "PATH=$ENV{SystemRoot}/System32"
  "${prefix}/bin/rtc_console.exe" --help
  WORKING_DIRECTORY "${BUILD_DIR}"
  RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error TIMEOUT 10)
if(NOT result EQUAL 0 OR NOT output MATCHES "Usage:")
  message(FATAL_ERROR "Installed console cannot start: ${result}\n${output}${error}")
endif()
