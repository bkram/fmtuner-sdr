#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <cstdint>

struct Config {
    struct RTLTCPSection {
        std::string host = "localhost";
        uint16_t port = 1234;
    } rtl_tcp;

    struct AudioSection {
        std::string device;
        int output_rate = 32000;
        int buffer_size = 1024;
        int underflow_fade_ms = 5;
        bool click_suppressor = false;
    } audio;

    struct SDRSection {
        int rtl_gain_db = -1;
        int default_custom_gain_flags = 0;
        bool overload_auto_gain = true;
        int overload_auto_gain_max_db = 49;
        std::string gain_strategy = "tef"; // tef|sdrpp
        bool sdrpp_rtl_agc = false;
        double signal_floor_dbfs = -55.0;
        double signal_ceil_dbfs = -19.0;
    } sdr;

    struct TunerSection {
        uint32_t default_freq = 88600;
        int deemphasis = 0;
    } tuner;

    struct XDRSection {
        uint16_t port = 7373;
        std::string password;
        bool guest_mode = false;
    } xdr;

    struct ProcessingSection {
        int agc_mode = 2;
        bool client_gain_allowed = true;
        std::string demodulator = "exact";
        std::string stereo_blend = "normal";
        bool stereo = true;
        bool rds = true;
    } processing;

    struct RDSSection {
        std::string aggressiveness = "balanced";
        float agc_attack = 0.995f;
        float agc_release = 0.9995f;
        int lock_acquire_groups = 2;
        int lock_loss_groups = 12;
    } rds;

    struct DebugSection {
        int log_level = 1;
    } debug;

    struct ReconnectionSection {
        bool auto_reconnect = true;
    } reconnection;

    bool loadFromFile(const std::string& filename);
    void loadDefaults();
};

#endif
