# Signal Strength Calculation - Flaw Analysis and Fix

## Current Implementation

The signal strength is calculated in `main.cpp` (lines 1137-1163):

```cpp
// RF-domain strength estimate from raw IQ power before demodulation.
const double powerSum = computeNormalizedIqPowerSum(iqBuffer, samples);
const double avgPower = (samples > 0) ? (powerSum / static_cast<double>(samples)) : 0.0;
const double dbfs = 10.0 * std::log10(avgPower + 1e-12);
...
const double gainCompDb = static_cast<double>(effectiveAppliedGainDb());
const double compensatedDbfs = dbfs - gainCompDb;
const double kRfFloorDbfs = config.sdr.signal_floor_dbfs;
const double kRfCeilDbfs = std::max(config.sdr.signal_ceil_dbfs, kRfFloorDbfs + 1.0);
const double norm = (compensatedDbfs - kRfFloorDbfs) / (kRfCeilDbfs - kRfFloorDbfs);
const float rfLevel = std::clamp(static_cast<float>(norm * 90.0), 0.0f, 90.0f);
```

## Flaws Identified

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

---

### Flaw 2: Incorrect Gain Compensation Direction (Medium Severity)

**Problem:** The code subtracts `gainCompDb` from dBFS to "compensate" for tuner gain:
```cpp
const double compensatedDbfs = dbfs - gainCompDb;
```

However, RTL-SDR applies gain at the **RF/tuner stage before ADC**, so the IQ samples already contain the amplified signal. The raw dBFS already reflects the signal **after** the tuner gain has been applied.

**Impact:** This creates **double compensation** - the gain is already baked into the IQ samples, but we're subtracting it again. The reported signal strength is artificially low by the gain amount.

**Fix:** Remove the gain compensation entirely, or only compensate if gain is applied in baseband after ADC:
```cpp
// Use raw dBFS without compensation (gain is already in the IQ data)
const double compensatedDbfs = dbfs;
```

---

### Flaw 3: Linear Mapping on Logarithmic Scale (Low Severity)

**Problem:** The normalization uses a linear mapping on a logarithmic dB scale:
```cpp
const double norm = (compensatedDbfs - kRfFloorDbfs) / (kRfCeilDbfs - kRfFloorDbfs);
```

This provides poor resolution at low signal levels and compresses the useful range.

**Impact:** The 0-90 scale doesn't represent signal strength linearly in a perceptually meaningful way.

**Fix:** Consider using a dB-linear mapping or a piecewise approach:
```cpp
// Option A: Keep dB scale but map more usefully
// Map -100dB to 0, -20dB to 90 (typical FM range)
const double minDb = -100.0;
const double maxDb = -20.0;
const double norm = std::clamp((compensatedDbfs - minDb) / (maxDb - minDb), 0.0, 1.0);
const float rfLevel = static_cast<float>(norm * 90.0);
```

---

### Flaw 4: Hardcoded 90 Scale Inconsistency (Low Severity)

**Problem:** The output scale is hardcoded to 0-90 but the XDR protocol and comments reference other scales. The value 90 seems arbitrary.

**Fix:** Document or make consistent with XDR protocol expectations (typically 0-100 or 0-255).

---

## Recommended Fix

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

---

## Summary of Issues

| Issue | Severity | Current | Fix |
|-------|----------|---------|-----|
| Wrong divisor in avgPower | Medium | `/ samples` | `/ (samples * 2)` |
| Gain compensation | Medium | `dbfs - gain` | Remove (gain already in IQ) |
| Linear on log scale | Low | Linear norm | Use dB range mapping |
| Hardcoded 90 scale | Low | Fixed | Document or make configurable |
