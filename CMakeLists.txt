cmake_minimum_required(VERSION 3.16)

project(
  LowCostSingleHipDOFC
  VERSION 0.1.0
  DESCRIPTION "Low-cost single-hip delayed output feedback control framework"
  LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(dofc_core
  src/control/DofcController.cpp
  src/core/DelayBuffer.cpp
  src/core/MotionStateDetector.cpp
  src/core/SafetyLimiter.cpp
  src/estimation/SeriesElasticEstimator.cpp
  src/io/CsvLogger.cpp
  src/model/SingleHipModel.cpp
)

target_include_directories(dofc_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(MSVC)
  target_compile_options(dofc_core PRIVATE /W4 /permissive-)
else()
  target_compile_options(dofc_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(simulate_dofc apps/simulate_dofc.cpp)
target_link_libraries(simulate_dofc PRIVATE dofc_core)

add_executable(parameter_sweep apps/parameter_sweep.cpp)
target_link_libraries(parameter_sweep PRIVATE dofc_core)

enable_testing()

add_executable(dofc_tests
  tests/test_delay_buffer.cpp
  tests/test_dofc_controller.cpp
  tests/test_full_scale_model.cpp
  tests/test_safety_limiter.cpp
  tests/test_series_elastic_estimator.cpp
  tests/test_main.cpp
)
target_link_libraries(dofc_tests PRIVATE dofc_core)

add_test(NAME dofc_tests COMMAND dofc_tests)
