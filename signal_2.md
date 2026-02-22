# Signal Strength Calculation - Detailed Analysis and Fix

## Current Implementation Analysis

The signal strength calculation in `main.cpp` (lines 1137-1163) has multiple fundamental flaws that affect accuracy and reliability.

### Flaw 1: Incorrect Average Power Calculation (Medium Severity)

**Problem:** The code divides `powerSum` by `samples` but `powerSum` contains the sum of squared values for **both** I and Q samples (total `samples * 2` values).

```cpp
const double powerSum = computeNormalizedIqPowerSum(iqBuffer, samples);
const double avgPower = (samples > 0) ? (powerSum / static_cast<double>(samples)) : 0.0;
```

**Impact:** The average power is **2x higher** than it should be, resulting in dBFS being approximately **3dB too high**.

**Fix:**
```cpp
const double avgPower = (samples > 0) ? (powerSum / static_cast<double>(samples * 2)) : 0.0;
```

### Flaw 2: Incorrect Gain Compensation Direction (Medium Severity)

**Problem:** The code subtracts `gainCompDb` from dBFS to "compensate" for tuner gain:
```cpp
const double compensatedDbfs = dbfs - gainCompDb;
```

However, RTL-SDR applies gain at the **RF/tuner stage before ADC**, so the IQ samples already contain the amplified signal. The raw dBFS already reflects the signal **after** the tuner gain has been applied.

**Impact:** This creates **double compensation** - the gain is already baked into the IQ samples, but we're subtracting it again. The reported signal strength is artificially low by the gain amount.

**Fix:** Remove the gain compensation entirely:
```cpp
const double compensatedDbfs = dbfs;
```

### Flaw 3: Linear Mapping on Logarithmic Scale (Low Severity)

**Problem:** The normalization uses a linear mapping on a logarithmic dB scale:
```cpp
const double norm = (compensatedDbfs - kRfFloorDbfs) / (kRfCeilDbfs - kRfFloorDbfs);
```

This provides poor resolution at low signal levels and compresses the useful range.

**Impact:** The 0-90 scale doesn't represent signal strength linearly in a perceptually meaningful way.

**Fix:** Use a dB-linear mapping with sensible FM range:
```cpp
const double minDb = -100.0;
const double maxDb = -20.0;
const double norm = std::clamp((compensatedDbfs - minDb) / (maxDb - minDb), 0.0, 1.0);
const float rfLevel = static_cast<float>(norm * 90.0);
```

### Flaw 4: Hardcoded 90 Scale Inconsistency (Low Severity)

**Problem:** The output scale is hardcoded to 0-90 but the XDR protocol and comments reference other scales. The value 90 seems arbitrary.

**Fix:** Document or make consistent with XDR protocol expectations (typically 0-100 or 0-255).

## Recommended Fix Implementation

Replace the signal strength calculation block (lines 1137-1163) with:

```cpp
// RF-domain strength estimate from raw IQ power before demodulation.
const double powerSum = computeNormalizedIqPowerSum(iqBuffer, samples);
// FIX: Divide by samples*2 since powerSum contains both I and Q values
const double avgPower = (samples > 0) ? (powerSum / static_cast<double>(samples * 2)) : 0.0;
const double dbfs = 10.0 * std::log10(avgPower + 1e-12);

// FIX: Remove gain compensation - gain is already applied in IQ samples by tuner
// The raw dBFS represents the signal level at the ADC
const double compensatedDbfs = dbfs;

// Map dBFS to 0-90 scale using sensible FM range (-100 to -20 dBFS)
const double kRfFloorDbfs = -100.0;  // Typical noise floor
const double kRfCeilDbfs = -20.0;   // Strong signal ceiling
const double norm = std::clamp((compensatedDbfs - kRfFloorDbfs) / (kRfCeilDbfs - kRfFloorDbfs), 0.0, 1.0);
const float rfLevel = static_cast<float>(norm * 90.0);
```

## Summary of Issues and Fixes

| Issue | Severity | Current | Fix |
|-------|----------|---------|-----|
| Wrong divisor in avgPower | Medium | `/ samples` | `/ (samples * 2)` |
| Gain compensation | Medium | `dbfs - gain` | Remove (gain already in IQ) |
| Linear on log scale | Low | Linear norm | Use dB range mapping |
| Hardcoded 90 scale | Low | Fixed | Document or make configurable |

## Testing Considerations

After applying the fix:
1. Verify signal strength readings are now accurate (not 3dB too high)
2. Check that gain adjustments don't double-compensate
3. Ensure the 0-90 scale represents actual signal levels meaningfully
4. Test with various signal strengths from noise floor to strong signals