#include "audio_output.h"
#include <iostream>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <cctype>
#include <chrono>

#ifdef __APPLE__
namespace {
std::string trim(const std::string& value) {
    size_t start = 0;
    while (start < value.size() && std::isspace(static_cast<unsigned char>(value[start])) != 0) {
        start++;
    }
    size_t end = value.size();
    while (end > start && std::isspace(static_cast<unsigned char>(value[end - 1])) != 0) {
        end--;
    }
    return value.substr(start, end - start);
}

std::string normalizeSelector(const std::string& rawSelector) {
    std::string selector = trim(rawSelector);
    if (selector.size() >= 2) {
        const char first = selector.front();
        const char last = selector.back();
        if ((first == '"' && last == '"') || (first == '\'' && last == '\'')) {
            selector = trim(selector.substr(1, selector.size() - 2));
        }
    }
    return selector;
}

std::string toLower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

bool parseDeviceIndex(const std::string& selector, int& outIndex) {
    if (selector.empty()) {
        return false;
    }
    for (char c : selector) {
        if (c < '0' || c > '9') {
            return false;
        }
    }
    outIndex = std::stoi(selector);
    return true;
}

PaDeviceIndex selectOutputDevice(const std::string& selector) {
    if (selector.empty()) {
        return Pa_GetDefaultOutputDevice();
    }

    int requestedIndex = -1;
    if (parseDeviceIndex(selector, requestedIndex)) {
        const int deviceCount = Pa_GetDeviceCount();
        if (requestedIndex >= 0 && requestedIndex < deviceCount) {
            const PaDeviceInfo* info = Pa_GetDeviceInfo(requestedIndex);
            if (info && info->maxOutputChannels > 0) {
                return static_cast<PaDeviceIndex>(requestedIndex);
            }
        }
        return paNoDevice;
    }

    const std::string needle = toLower(selector);
    const int deviceCount = Pa_GetDeviceCount();
    for (int i = 0; i < deviceCount; i++) {
        const PaDeviceInfo* info = Pa_GetDeviceInfo(i);
        if (!info || info->maxOutputChannels <= 0 || !info->name) {
            continue;
        }
        if (toLower(info->name).find(needle) != std::string::npos) {
            return static_cast<PaDeviceIndex>(i);
        }
    }

    return paNoDevice;
}

void printOutputDeviceList() {
    const int deviceCount = Pa_GetDeviceCount();
    std::cerr << "Available PortAudio output devices:\n";
    for (int i = 0; i < deviceCount; i++) {
        const PaDeviceInfo* info = Pa_GetDeviceInfo(i);
        if (!info || info->maxOutputChannels <= 0 || !info->name) {
            continue;
        }
        std::cerr << "  [" << i << "] " << info->name << "\n";
    }
}
}  // namespace
#endif

#ifdef __APPLE__
void AudioOutput::runOutputThread() {
    constexpr size_t kBlockSamples = static_cast<size_t>(FRAMES_PER_BUFFER) * CHANNELS;
    std::vector<float> block(kBlockSamples, 0.0f);
    float lastL = 0.0f;
    float lastR = 0.0f;

    while (m_outputThreadRunning.load()) {
        size_t copied = 0;
        {
            std::unique_lock<std::mutex> lock(m_outputMutex);
            m_outputCv.wait_for(lock, std::chrono::milliseconds(10), [&]() {
                const size_t available = (m_outputQueue.size() > m_outputReadIndex)
                    ? (m_outputQueue.size() - m_outputReadIndex)
                    : 0;
                return !m_outputThreadRunning.load() || available > 0;
            });

            if (!m_outputThreadRunning.load()) {
                break;
            }

            const size_t available = (m_outputQueue.size() > m_outputReadIndex)
                ? (m_outputQueue.size() - m_outputReadIndex)
                : 0;
            copied = std::min(available, kBlockSamples);
            if (copied > 0) {
                std::memcpy(block.data(), m_outputQueue.data() + m_outputReadIndex, copied * sizeof(float));
                m_outputReadIndex += copied;
            }

            if (copied >= 2) {
                lastL = block[copied - 2];
                lastR = block[copied - 1];
            }

            if (copied < kBlockSamples) {
                const size_t missing = kBlockSamples - copied;
                const int fadeMs = std::clamp(m_underflowFadeMs.load(std::memory_order_relaxed), 0, 50);
                const size_t fadeSamples = std::min(
                    missing,
                    static_cast<size_t>(SAMPLE_RATE * (static_cast<float>(fadeMs) / 1000.0f) * CHANNELS));
                for (size_t i = 0; i < fadeSamples; i += 2) {
                    const float t = static_cast<float>(i / 2) /
                        static_cast<float>(std::max<size_t>(1, (fadeSamples / 2) - 1));
                    const float gain = 1.0f - t;
                    const size_t idx = copied + i;
                    block[idx] = lastL * gain;
                    if (idx + 1 < kBlockSamples) {
                        block[idx + 1] = lastR * gain;
                    }
                }
                for (size_t i = copied + fadeSamples; i < kBlockSamples; i++) {
                    block[i] = 0.0f;
                }
            }

            if (m_outputReadIndex > 0 && (m_outputReadIndex >= 16384 || (m_outputReadIndex * 2) >= m_outputQueue.size())) {
                m_outputQueue.erase(m_outputQueue.begin(), m_outputQueue.begin() + static_cast<std::ptrdiff_t>(m_outputReadIndex));
                m_outputReadIndex = 0;
            }
        }

        const PaError writeErr = Pa_WriteStream(m_paStream, block.data(), FRAMES_PER_BUFFER);
        if (writeErr == paOutputUnderflowed) {
            static std::atomic<uint32_t> underflowCount{0};
            const uint32_t count = ++underflowCount;
            if (m_verboseLogging && (count <= 5 || (count % 50) == 0)) {
                std::cerr << "[Audio] output underflow (" << count
                          << ") - consider higher system audio buffer/less CPU load\n";
            }
        } else if (writeErr != paNoError) {
            if (m_verboseLogging) {
                std::cerr << "[Audio] write failed: " << Pa_GetErrorText(writeErr) << "\n";
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
}
#endif

AudioOutput::AudioOutput()
    : m_enableSpeaker(false)
    , m_wavHandle(nullptr)
    , m_running(false)
    , m_wavDataSize(0)
    , m_writeIndex(0)
    , m_readIndex(0)
    , m_verboseLogging(true)
    , m_requestedVolumePercent(100)
    , m_underflowFadeMs(5)
    , m_currentVolumeScale(0.85f)
#ifdef __APPLE__
    , m_paStream(nullptr)
    , m_portAudioInitialized(false)
    , m_outputThreadRunning(false)
    , m_outputReadIndex(0)
#endif
{
    m_circularBuffer.resize(65536 * 2);
}

AudioOutput::~AudioOutput() {
    shutdown();
}

bool AudioOutput::init(bool enableSpeaker, const std::string& wavFile, const std::string& deviceSelector, bool verboseLogging) {
    m_enableSpeaker = enableSpeaker;
    m_wavFile = wavFile;
    m_verboseLogging = verboseLogging;

    if (!wavFile.empty()) {
        if (!initWAV(wavFile)) {
            std::cerr << "Failed to initialize WAV file" << std::endl;
            return false;
        }
    }

#ifdef __APPLE__
    if (enableSpeaker) {
        const std::string normalizedSelector = normalizeSelector(deviceSelector);
        if (verboseLogging) {
            std::cerr << "[Audio] device selector raw='" << deviceSelector
                      << "' normalized='" << normalizedSelector << "'\n";
        }
        PaError err = Pa_Initialize();
        if (err != paNoError) {
            std::cerr << "PortAudio init failed: " << Pa_GetErrorText(err) << std::endl;
            return false;
        } else {
            m_portAudioInitialized = true;
            PaStreamParameters outputParams;
            outputParams.device = selectOutputDevice(normalizedSelector);
            if (outputParams.device == paNoDevice) {
                std::cerr << "PortAudio device not found for selector: " << normalizedSelector << std::endl;
                printOutputDeviceList();
                Pa_Terminate();
                m_portAudioInitialized = false;
                return false;
            }
            outputParams.channelCount = CHANNELS;
            outputParams.sampleFormat = paFloat32;
            const PaDeviceInfo* deviceInfo = Pa_GetDeviceInfo(outputParams.device);
            if (!deviceInfo) {
                std::cerr << "PortAudio failed to get device info for selected output device" << std::endl;
                Pa_Terminate();
                m_portAudioInitialized = false;
                return false;
            }
            // Favor stability over ultra-low latency to avoid audible underruns/clicks.
            outputParams.suggestedLatency = std::max(deviceInfo->defaultLowOutputLatency,
                                                     deviceInfo->defaultHighOutputLatency);
            outputParams.hostApiSpecificStreamInfo = nullptr;
            if (verboseLogging) {
                if (normalizedSelector.empty()) {
                    std::cerr << "Using default output device [" << outputParams.device << "]: " << deviceInfo->name << std::endl;
                } else {
                    std::cerr << "Using selected output device [" << outputParams.device << "]: " << deviceInfo->name << std::endl;
                }
            }

            err = Pa_OpenStream(&m_paStream, nullptr, &outputParams, SAMPLE_RATE, FRAMES_PER_BUFFER, paClipOff, nullptr, nullptr);
            if (err != paNoError) {
                std::cerr << "PortAudio open failed: " << Pa_GetErrorText(err) << std::endl;
                Pa_Terminate();
                m_portAudioInitialized = false;
                return false;
            } else {
                err = Pa_StartStream(m_paStream);
                if (err != paNoError) {
                    std::cerr << "PortAudio start failed: " << Pa_GetErrorText(err) << std::endl;
                    Pa_CloseStream(m_paStream);
                    m_paStream = nullptr;
                    Pa_Terminate();
                    m_portAudioInitialized = false;
                    return false;
                } else if (verboseLogging) {
                    std::cerr << "PortAudio started successfully" << std::endl;
                }

                // Prime the output device with silence to reduce startup underruns.
                float primeBuffer[FRAMES_PER_BUFFER * CHANNELS] = {0.0f};
                for (int i = 0; i < 2; i++) {
                    const PaError primeErr = Pa_WriteStream(m_paStream, primeBuffer, FRAMES_PER_BUFFER);
                    if (primeErr != paNoError && primeErr != paOutputUnderflowed) {
                        std::cerr << "PortAudio prime failed: " << Pa_GetErrorText(primeErr) << std::endl;
                        Pa_StopStream(m_paStream);
                        Pa_CloseStream(m_paStream);
                        m_paStream = nullptr;
                        Pa_Terminate();
                        m_portAudioInitialized = false;
                        return false;
                    }
                }

                m_outputThreadRunning = true;
                m_outputThread = std::thread(&AudioOutput::runOutputThread, this);
            }
        }
    }
#endif

    m_running = true;
    return true;
}

void AudioOutput::shutdown() {
    m_running = false;

#ifdef __APPLE__
    m_outputThreadRunning = false;
    m_outputCv.notify_all();
    if (m_outputThread.joinable()) {
        m_outputThread.join();
    }
    if (m_paStream) {
        Pa_StopStream(m_paStream);
        Pa_CloseStream(m_paStream);
        m_paStream = nullptr;
    }
    if (m_portAudioInitialized) {
        Pa_Terminate();
        m_portAudioInitialized = false;
    }
#endif

    closeWAV();
}

void AudioOutput::setVolumePercent(int volumePercent) {
    m_requestedVolumePercent.store(std::clamp(volumePercent, 0, 100), std::memory_order_relaxed);
}

void AudioOutput::setUnderflowFadeMs(int fadeMs) {
    m_underflowFadeMs.store(std::clamp(fadeMs, 0, 50), std::memory_order_relaxed);
}

bool AudioOutput::initWAV(const std::string& filename) {
    m_wavHandle = fopen(filename.c_str(), "wb");
    if (!m_wavHandle) {
        return false;
    }

    m_wavDataSize = 0;
    writeWAVHeader();
    return true;
}

void AudioOutput::writeWAVHeader() {
    if (!m_wavHandle) return;

    uint32_t sampleRate = SAMPLE_RATE;
    uint16_t numChannels = CHANNELS;
    uint16_t bitsPerSample = BITS_PER_SAMPLE;
    uint32_t byteRate = sampleRate * numChannels * bitsPerSample / 8;
    uint16_t blockAlign = numChannels * bitsPerSample / 8;
    uint32_t dataSize = m_wavDataSize;

    fseek(m_wavHandle, 0, SEEK_SET);

    fwrite("RIFF", 1, 4, m_wavHandle);
    uint32_t fileSize = 36 + dataSize;
    fwrite(&fileSize, 4, 1, m_wavHandle);
    fwrite("WAVE", 1, 4, m_wavHandle);

    fwrite("fmt ", 1, 4, m_wavHandle);
    uint32_t fmtSize = 16;
    fwrite(&fmtSize, 4, 1, m_wavHandle);
    uint16_t audioFormat = 1;
    fwrite(&audioFormat, 2, 1, m_wavHandle);
    fwrite(&numChannels, 2, 1, m_wavHandle);
    fwrite(&sampleRate, 4, 1, m_wavHandle);
    fwrite(&byteRate, 4, 1, m_wavHandle);
    fwrite(&blockAlign, 2, 1, m_wavHandle);
    fwrite(&bitsPerSample, 2, 1, m_wavHandle);

    fwrite("data", 1, 4, m_wavHandle);
    fwrite(&dataSize, 4, 1, m_wavHandle);
}

bool AudioOutput::writeWAVData(const float* left, const float* right, size_t numSamples) {
    if (!m_wavHandle) return false;

    std::vector<int16_t> buffer(numSamples * CHANNELS);

    for (size_t i = 0; i < numSamples; i++) {
        float l = std::max(-1.0f, std::min(1.0f, left[i]));
        float r = std::max(-1.0f, std::min(1.0f, right[i]));
        buffer[i * 2] = static_cast<int16_t>(l * 32767.0f);
        buffer[i * 2 + 1] = static_cast<int16_t>(r * 32767.0f);
    }

    size_t written = fwrite(buffer.data(), sizeof(int16_t), numSamples * CHANNELS, m_wavHandle);
    m_wavDataSize += written * sizeof(int16_t);

    return written == numSamples * CHANNELS;
}

void AudioOutput::closeWAV() {
    if (m_wavHandle) {
        writeWAVHeader();
        fclose(m_wavHandle);
        m_wavHandle = nullptr;
    }
}

bool AudioOutput::write(const float* left, const float* right, size_t numSamples) {
    if (!m_running) return false;

    std::vector<float> scaledLeft;
    std::vector<float> scaledRight;
    const float* writeLeft = left;
    const float* writeRight = right;

    if (left && right && numSamples > 0) {
        scaledLeft.resize(numSamples);
        scaledRight.resize(numSamples);
        const float targetVolumeScale =
            (static_cast<float>(m_requestedVolumePercent.load(std::memory_order_relaxed)) / 100.0f) * 0.85f;
        const float rampSamples = static_cast<float>(SAMPLE_RATE) * 0.01f;
        const float step = (targetVolumeScale - m_currentVolumeScale) / std::max(1.0f, rampSamples);

        for (size_t i = 0; i < numSamples; i++) {
            if (std::abs(targetVolumeScale - m_currentVolumeScale) > 1e-6f) {
                m_currentVolumeScale += step;
                if ((step > 0.0f && m_currentVolumeScale > targetVolumeScale) ||
                    (step < 0.0f && m_currentVolumeScale < targetVolumeScale)) {
                    m_currentVolumeScale = targetVolumeScale;
                }
            }
            scaledLeft[i] = left[i] * m_currentVolumeScale;
            scaledRight[i] = right[i] * m_currentVolumeScale;
        }
        writeLeft = scaledLeft.data();
        writeRight = scaledRight.data();
    }

    if (m_wavHandle) {
        writeWAVData(writeLeft, writeRight, numSamples);
    }

#ifdef __APPLE__
    if (m_enableSpeaker && m_paStream) {
        std::lock_guard<std::mutex> lock(m_outputMutex);
        constexpr size_t kMaxQueuedSamples = static_cast<size_t>(SAMPLE_RATE) * CHANNELS * 2;
        const size_t queuedSamples = (m_outputQueue.size() > m_outputReadIndex)
            ? (m_outputQueue.size() - m_outputReadIndex)
            : 0;
        const size_t incomingSamples = numSamples * CHANNELS;
        if (queuedSamples + incomingSamples > kMaxQueuedSamples) {
            const size_t drop = (queuedSamples + incomingSamples) - kMaxQueuedSamples;
            const size_t samplesToRamp = std::min(drop, static_cast<size_t>(FRAMES_PER_BUFFER * CHANNELS * 4));
            if (samplesToRamp > 0) {
                const size_t startIdx = m_outputQueue.size() - samplesToRamp;
                const size_t channels = CHANNELS;
                for (size_t i = 0; i < samplesToRamp; i++) {
                    const float ramp = static_cast<float>(samplesToRamp - i) / static_cast<float>(samplesToRamp);
                    m_outputQueue[startIdx + i] *= ramp;
                }
            }
            m_outputReadIndex += std::min(drop, queuedSamples);
            if (m_verboseLogging) {
                static std::atomic<uint32_t> dropCount{0};
                const uint32_t count = ++dropCount;
                if (count <= 5 || (count % 50) == 0) {
                    std::cerr << "[Audio] queue overflow (" << count
                              << ") - ramping out oldest samples to prevent clicks\n";
                }
            }
        }

        const size_t base = m_outputQueue.size();
        m_outputQueue.resize(base + incomingSamples);
        for (size_t i = 0; i < numSamples; i++) {
            m_outputQueue[base + i * 2] = writeLeft[i];
            m_outputQueue[base + i * 2 + 1] = writeRight[i];
        }
        m_outputCv.notify_one();
    }
#endif

    return true;
}
