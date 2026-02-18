#ifndef AF_POST_PROCESSOR_H
#define AF_POST_PROCESSOR_H

#include <stddef.h>
#include <stdint.h>
#include <vector>

class AFPostProcessor {
public:
    AFPostProcessor(int inputRate, int outputRate);

    void reset();
    void setDeemphasis(int tau_us);

    size_t process(const float* inLeft,
                   const float* inRight,
                   size_t inSamples,
                   float* outLeft,
                   float* outRight,
                   size_t outCapacity);

private:
    float sinc(float x) const;
    void designPolyphaseKernel();
    float convolveDot(const float* samples, const float* taps, size_t count) const;
    void processDCBlock(float* left, float* right, size_t samples);

    int m_inputRate;
    int m_outputRate;
    int m_upFactor;
    int m_downFactor;
    int m_halfTaps;
    int m_tapCount;
    std::vector<float> m_phaseTaps;
    std::vector<float> m_leftBuffer;
    std::vector<float> m_rightBuffer;
    size_t m_bufferStart;
    int64_t m_timeAcc;
    bool m_useNeon;
    bool m_useSse2;
    bool m_useAvx2;

    bool m_deemphasisEnabled;
    float m_deemphAlpha;
    float m_deemphStateLeft;
    float m_deemphStateRight;

    float m_dcBlockPrevInLeft;
    float m_dcBlockPrevInRight;
    float m_dcBlockPrevOutLeft;
    float m_dcBlockPrevOutRight;
    static constexpr float kDcBlockR = 0.995f;
};

#endif
