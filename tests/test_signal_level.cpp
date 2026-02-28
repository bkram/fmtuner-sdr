#include <catch2/catch_test_macros.hpp>
#include <cstring>
#include "signal_level.h"

TEST_CASE("computeSignalLevel handles null input", "[signal_level]") {
    SignalLevelResult result = computeSignalLevel(nullptr, 100, 0, 0.5, 0.0, -80.0, -12.0);
    REQUIRE(result.dbfs == -120.0);
    REQUIRE(result.level120 == 0.0f);
}

TEST_CASE("computeSignalLevel handles zero samples", "[signal_level]") {
    uint8_t buffer[256];
    SignalLevelResult result = computeSignalLevel(buffer, 0, 0, 0.5, 0.0, -80.0, -12.0);
    REQUIRE(result.dbfs == -120.0);
    REQUIRE(result.level120 == 0.0f);
}

TEST_CASE("computeSignalLevel computes level for silent input", "[signal_level]") {
    uint8_t buffer[256];
    std::memset(buffer, 127, sizeof(buffer));
    
    SignalLevelResult result = computeSignalLevel(buffer, 128, 0, 0.5, 0.0, -80.0, -12.0);
    REQUIRE(result.dbfs < -60.0);
    REQUIRE(result.level120 == 0.0f);
}

TEST_CASE("smoothSignalLevel initializes on first call", "[signal_level]") {
    SignalLevelSmoother state;
    REQUIRE(state.initialized == false);
    
    float result = smoothSignalLevel(50.0f, state);
    REQUIRE(result == 50.0f);
    REQUIRE(state.initialized == true);
    REQUIRE(state.value == 50.0f);
}

TEST_CASE("smoothSignalLevel smooths subsequent values", "[signal_level]") {
    SignalLevelSmoother state;
    state.initialized = true;
    state.value = 50.0f;
    
    float result = smoothSignalLevel(60.0f, state);
    REQUIRE(result > 50.0f);
    REQUIRE(result < 60.0f);
}

TEST_CASE("computeSignalLevel reports clip ratio", "[signal_level]") {
    uint8_t buffer[256];
    std::memset(buffer, 0, sizeof(buffer));
    
    SignalLevelResult result = computeSignalLevel(buffer, 128, 0, 0.5, 0.0, -80.0, -12.0);
    REQUIRE(result.hardClipRatio > 0.0);
}
