# =============================================================================
# hf-tmc9660-driver — Root CMakeLists.txt
# =============================================================================
# Platform-agnostic build for the HardFOC TMC9660 driver.
# For ESP-IDF builds use the component wrapper instead.
# =============================================================================
cmake_minimum_required(VERSION 3.16)

project(hf_tmc9660
  VERSION   1.0.0
  LANGUAGES CXX
)

# ── Import shared build settings ─────────────────────────────────────────────
include(${CMAKE_CURRENT_LIST_DIR}/cmake/hf_tmc9660_build_settings.cmake)

# ── Library target ───────────────────────────────────────────────────────────
add_library(${HF_TMC9660_TARGET_NAME} STATIC ${HF_TMC9660_SOURCE_FILES})

target_include_directories(${HF_TMC9660_TARGET_NAME} PUBLIC ${HF_TMC9660_PUBLIC_INCLUDE_DIRS})

target_compile_features(${HF_TMC9660_TARGET_NAME} PUBLIC cxx_std_20)

# ── Optional warnings (off by default, enable with -DHF_TMC9660_ENABLE_WARNINGS=ON)
option(HF_TMC9660_ENABLE_WARNINGS "Enable compiler warnings" OFF)
if(HF_TMC9660_ENABLE_WARNINGS)
  target_compile_options(${HF_TMC9660_TARGET_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ── Host unit tests (mock transports; no hardware) ───────────────────────────
option(HF_TMC9660_BUILD_TESTS "Build the host unit tests in tests/" OFF)
if(HF_TMC9660_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

# ── CMake package export ─────────────────────────────────────────────────────
include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/hf_tmc9660ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/hf_tmc9660Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/hf_tmc9660Config.cmake"
  INSTALL_DESTINATION lib/cmake/hf_tmc9660
)
