# FM-SDR-Tuner Issues and Improvements

## Static Analysis Issues

### Critical
- **src/xdr_server.cpp:325-340**: Memory leak - SHA1 context not destroyed on failure
- **src/rtl_sdr_device.cpp:216-218**: Potential null dereference - `m_deviceHandle` not validated in all paths
- **src/audio_output.cpp:1265-1324**: Potential null dereference - `m_wavHandle` not checked before writing

### High
- **src/xdr_server.cpp:325-340**: Missing error checking for OpenSSL functions (SHA1_Init, SHA1_Update, SHA1_Final)
- **src/rtl_tcp_client.cpp:142-148**: Header read without proper validation
- **src/main.cpp:1640-1641**: Vector resize without allocation failure check

### Medium
- **src/main.cpp:1320-1324**: Unused parameter `appliedGainDb` in signal function
- **src/main.cpp:1340-1338**: Verbose logging variables declared but unused in non-verbose mode
- **src/xdr_server.cpp:1326**: Unused parameter `arg` in processFmdxCommand

### Low
- **src/rtl_sdr_device.cpp:97-100**: Missing error handling for rtlsdr_get_device_count() failure

---

## Design Issues

### Critical
- **src/main.cpp**: 1500+ line main function with massive coupling - difficult to test and maintain
- Real-time audio paths have blocking I/O (WAV writing, network I/O)

### High
- **src/audio_output.cpp**: No abstraction for audio backends (PortAudio/WAV)
- **src/rtl_sdr_device.cpp**: RTL-SDR specific code not abstracted behind device interface
- **src/rtl_tcp_client.cpp**: Mixed error handling (bool returns vs exceptions)

### Medium
- **src/config.cpp:24-28** and **src/config.cpp:54-58**: Duplicate string transformation functions
- **src/main.cpp:1370-1380**: Volume scaling logic duplicated
- **src/xdr_server.cpp:1249-1263** and **src/xdr_server.cpp:1314-1326**: Duplicate callback setter patterns

### Low
- **src/main.cpp**: Mixed naming conventions (camelCase vs snake_case)
- **src/xdr_server.cpp**: Inconsistent variable naming (m_ vs no prefix)
- Many member functions should be const-correct but aren't

---

## Performance Issues

### High
- **src/main.cpp:1318-1325**: Signal level computation in every loop iteration
- **src/rtl_sdr_device.cpp:212-274**: readIQ can block for 35ms
- **src/audio_output.cpp:1334-1356**: WAV file I/O in audio callback (real-time path)
- **src/xdr_server.cpp:602-641**: Network I/O in audio processing path

### Medium
- **src/main.cpp:1094-1100**: Multiple vector allocations in audio processing loop
- **src/stereo_decoder.cpp:230-267**: Pilot detection algorithm could be optimized

---

## Maintainability Issues

### Medium
- **src/main.cpp:1089**: Hardcoded decimation factors (magic numbers)
- **src/stereo_decoder.cpp:5-22**: Magic constants for PLL parameters
- **src/audio_output.cpp:31-34**: Hardcoded audio parameters

### Low
- Signal level computation doesn't handle invalid input gracefully
- Inconsistent error handling strategies across modules

---

## Priority Recommendations

1. **Immediate**: Fix memory leak in xdr_server.cpp SHA1 usage
2. **Immediate**: Add null checks for device/file handles
3. **High**: Refactor main.cpp into smaller modules/functions
4. **High**: Abstract audio output behind interface
5. **Medium**: Add const-correctness where appropriate
6. **Medium**: Replace magic numbers with named constants
7. **Low**: Standardize naming conventions

---

## clang-tidy Findings (~41,000 warnings)

### Critical (Errors)
- Missing headers (false positives): OpenSSL, liquid - build system issues

### High Priority Fixes
- **src/config.cpp:100**: Function cognitive complexity 204 (threshold 25) - refactor
- **src/af_post_processor.cpp:59-65**: Uninitialized variables (`leftProduced`, `rightProduced`, `left`, `right`)
- **src/audio_output.cpp:1314**: Potential null pointer dereference
- **src/audio_output.cpp:1266**: Assigning owner to non-owner (FILE*)

### Medium Priority Fixes
- **Magic numbers**: Replace with named constants
  - `75` (deemphasis tau) - af_post_processor.cpp:17
  - `100` (volume/buffer) - audio_output.cpp:930, 1262
  - `65536` - audio_output.cpp:953
  - `32767.0f` - audio_output.cpp:1316-1317
  - `0.85f`, `1e-6f` - audio_output.cpp:931, 1375
- **Short variable names**: Expand to meaningful names
  - `l`, `r` → `left`, `right` (audio_output.cpp:1314-1315)
  - `p` → `idx` or `sample` (af_post_processor.cpp:63)
  - `dt`, `b`, `a` → `deltaTime`, `coeffs`, `coeffs` (af_post_processor.cpp:39-42)
- **Pointer arithmetic**: Use array indexing
  - af_post_processor.cpp:72-73
  - audio_output.cpp:1314-1315, 1382-1383
- **Implicit bool conversions**: Use explicit nullptr checks
  - af_post_processor.cpp:53
  - audio_output.cpp:1267, 1277, 1309, 1366, 1389
- **Missing const**: Make member functions const where appropriate
  - af_post_processor.cpp:20, 47

### Low Priority (Style)
- Uppercase float suffixes (`1.0f` → `1.0F`)
- `#if defined` → `#ifdef`
- `std::endl` → `'\n'`
- Add braces around single-line statements
- Trailing return types

---

## clang-format Issues

Many formatting violations found. Run to fix:
```bash
clang-format -i src/*.cpp
```
