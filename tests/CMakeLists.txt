# =============================================================================
# hf-tmc9660-driver — host unit tests
# =============================================================================
# Builds the driver headers against mock SPI/UART transports and runs the tests
# with CTest. No hardware and no ESP-IDF needed:
#
#   cmake -S . -B build-host -DHF_TMC9660_BUILD_TESTS=ON
#   cmake --build build-host
#   ctest --test-dir build-host --output-on-failure
# =============================================================================

add_executable(hf_tmc9660_host_tests
  test_main.cpp
  test_frames.cpp
  test_transport.cpp
  test_driver.cpp
)

target_link_libraries(hf_tmc9660_host_tests PRIVATE ${HF_TMC9660_TARGET_NAME})
target_include_directories(hf_tmc9660_host_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(hf_tmc9660_host_tests PRIVATE cxx_std_20)
target_compile_options(hf_tmc9660_host_tests PRIVATE -Wall -Wextra)

add_test(NAME hf_tmc9660_host_tests COMMAND hf_tmc9660_host_tests)
