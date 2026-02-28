cmake_minimum_required(VERSION 3.15)

add_executable(test_signal_level test_signal_level.cpp
    ${CMAKE_SOURCE_DIR}/src/signal_level.cpp
)
target_include_directories(test_signal_level PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${Catch2_INCLUDE_DIRS}
)
target_link_libraries(test_signal_level PRIVATE
    Catch2::Catch2WithMain
    Threads::Threads
)

add_executable(test_config test_config.cpp
    ${CMAKE_SOURCE_DIR}/src/config.cpp
)
target_include_directories(test_config PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${Catch2_INCLUDE_DIRS}
)
target_link_libraries(test_config PRIVATE
    Catch2::Catch2WithMain
)

add_test(NAME signal_level COMMAND test_signal_level)
add_test(NAME config COMMAND test_config)
