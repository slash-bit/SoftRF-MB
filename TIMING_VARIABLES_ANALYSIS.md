# SoftRF Timing Variables Analysis

## Critical Issue Found
**Current Bug**: `RF_time` stays at -536810240 instead of incrementing. `ref_time_ms` appears to be set to `OurTime` value (seconds) instead of milliseconds value.

---

## Timing Variables Summary

### 1. **OurTime** (Time.cpp:24)
- **Type**: `time_t` (signed long, typically 32-bit)
- **Units**: **SECONDS** since Unix epoch (1970-01-01 00:00:00 UTC)
- **Range**: Currently showing ~1,771,669,854 (year 2026)
- **Initialization**: 0
- **Updated by**: Time_loop() when valid GPS fix received
- **How derived**: `makeTime(tm)` from GPS date/time data
- **Used in**: RF_loop() to set RF_time

### 2. **ref_time_ms** (Time.cpp:26)
- **Type**: `uint32_t`
- **Units**: ???  **AMBIGUOUS - NEEDS CLARIFICATION**
- **Current observed value**: Same as `OurTime` (e.g., 1,771,669,848) = **WRONG!**
- **Expected**: Should be milliseconds of last PPS (0-999 or 0-999999?)
- **Initialization**: 0
- **Updated by**: Time_loop() line 346: `ref_time_ms = base_time_ms = newtime;`
- **Issues**:
  - Gets set to `newtime` which appears to be in SECONDS, not milliseconds
  - Makes RF_loop unusable because comparisons like `if (now_ms < ref_time_ms)` are comparing apples to oranges
- **Questions for developer**:
  1. Should `ref_time_ms` be milliseconds or seconds?
  2. What should `newtime` contain (milliseconds or seconds)?
  3. What is the correct formula to set `ref_time_ms`?

### 3. **base_time_ms** (Time.cpp:25)
- **Type**: `uint32_t`
- **Units**: Milliseconds (but documentation says "at last verified PPS")
- **Initialization**: 0
- **Set alongside**: ref_time_ms in line 346
- **Purpose**: Unclear - same as ref_time_ms?
- **Question**: What is difference between `base_time_ms` and `ref_time_ms`?

### 4. **RF_time** (RF.cpp:49)
- **Type**: `time_t` (signed long)
- **Units**: **SECONDS** (should match OurTime)
- **Initialization**: Uninitialized (defaults to 0 in global scope)
- **Current value**: -536810240 (WRONG - should be ~1,771,669,854)
- **Updated by**: RF_loop() line 2853: `RF_time = OurTime;`
- **Also modified by**:
  - Line 2856: `--OurTime; --RF_time;` (if `now_ms < ref_time_ms`)
  - Line 2864: `++RF_time;` (if `ms_since_pps >= 1300`)
  - Line 2873: `--RF_time;` (if `ms_since_pps < 300`)
- **Purpose**: Used in set_protocol_for_slot() for protocol switching timing
- **Used for**: Calculating `sec_3_7_11_15 = ((RF_time & 0x03) == 0x03)` and `sec_3_11 = ((RF_time & 0x07) == 0x03)`

### 5. **now_ms** (Local variable in functions)
- **Type**: `uint32_t`
- **Units**: Milliseconds from `millis()`
- **Range**: 0 to 2^32 (wraps after ~49 days)
- **Initialized**: `uint32_t now_ms = millis();`
- **Used in**:
  - Time.cpp line 187: Time_loop()
  - RF.cpp line 2847: RF_loop() **BUT PRINTED AS 0 IN DEBUG OUTPUT** ⚠️
- **Issue**: Debug output shows `now_ms=0` which is impossible - suggests variable shadowing or incorrect printf format

### 6. **ms_since_pps** (RF.cpp:2861)
- **Type**: `uint32_t`
- **Units**: Milliseconds
- **Calculated as**: `ms_since_pps = now_ms - ref_time_ms;`
- **Range**: 0-1300ms (within one time slot)
- **Purpose**: Determines which RF slot we're in (0 or 1)
- **Problem**: Calculation is WRONG if `ref_time_ms` is in seconds instead of milliseconds
- **Used for**:
  - Line 2880: `if (ms_since_pps >= 380 && ms_since_pps < 800)` → Slot 0
  - Line 2896: `if (ms_since_pps >= 800 && ms_since_pps < 1300)` → Slot 1

### 7. **pps_btime_ms** (Multiple files)
- **Type**: `uint32_t`
- **Units**: Milliseconds
- **Source**: `SoC->get_PPS_TimeMarker()` (RF.cpp:2560, Time.cpp:274)
- **Purpose**: Timestamp of last PPS pulse
- **Range**: 0-999 (milliseconds within a second)
- **Used in**: Calculating `newtime` for `ref_time_ms`

### 8. **latest_Commit_Time** (Time.cpp)
- **Type**: `uint32_t`
- **Units**: Milliseconds from `millis()`
- **Set by**: Time.cpp line 272: `latest_Commit_Time = now_ms;` (when GPS fix received)
- **Purpose**: Timestamp when GPS data was last received
- **Used in**: Calculating corrections and adjustments for time

### 9. **newtime** (Time.cpp, local variable in DEBUG_SIMULATE block)
- **Type**: `uint32_t`
- **Units**: AMBIGUOUS - appears to be milliseconds, but possibly seconds?
- **Calculated as**:
  - In DEBUG_SIMULATE: Line 250: `pps_btime_ms + ADJ_FOR_FLARM_RECEPTION`
  - In normal mode: Line 278 or 301
- **Assigned to**: `ref_time_ms = base_time_ms = newtime;` (line 346)
- **Question for developer**: What units should `newtime` be in? Why is it assigned directly to `ref_time_ms`?

### 10. **time_corr_neg** (Time.cpp)
- **Type**: `uint32_t`
- **Units**: Milliseconds (offset from PPS to time)
- **Purpose**: Correction factor for GPS time delays
- **Question**: Does this need to be applied to `newtime` before assigning to `ref_time_ms`?

---

## Current Timing Flow (BROKEN)

```
Time_loop():
  1. Gets GPS fix with date/time
  2. Calculates OurTime = makeTime(tm) [SECONDS since 1970]
  3. Calculates newtime = pps_btime_ms + ADJ... [MILLISECONDS?]
  4. Sets: ref_time_ms = base_time_ms = newtime [LINE 346]
          ↓
          ✗ BUG: ref_time_ms gets newtime which is MILLISECONDS
                but OurTime is SECONDS - units don't match!

RF_loop():
  1. Reads: RF_time = OurTime [SECONDS]
  2. Calculates: ms_since_pps = now_ms - ref_time_ms
               ↓
               ✗ BUG: Subtracting SECONDS from milliseconds = wrong result
```

---

## Questions for Original Developer

1. **What are the correct units and purpose for each of these variables?**
   - `ref_time_ms` - should it be milliseconds or seconds?
   - `newtime` - should it be milliseconds or seconds?
   - `base_time_ms` - what is its purpose vs `ref_time_ms`?

2. **How should `ref_time_ms` be derived from GPS data?**
   - Currently: `ref_time_ms = newtime`
   - Should it be: `ref_time_ms = (OurTime * 1000) + (pps_btime_ms % 1000)`?
   - Or something else?

3. **Why is `newtime` assigned directly to `ref_time_ms` without unit conversion?**

4. **In RF_loop, is the calculation `ms_since_pps = now_ms - ref_time_ms` correct?**
   - Both operands should be in same units (milliseconds)
   - But currently they might not be

5. **What is the difference between `base_time_ms` and `ref_time_ms`?**
   - They're always set to the same value
   - Why maintain both?

6. **In the DEBUG output, why does `ref_time_ms` show the same value as `OurTime`?**
   - This is clearly wrong
   - What should it show instead?

---

## Suspected Root Cause

`newtime` is being calculated in MILLISECONDS but should be combined with OurTime (SECONDS) to properly track "seconds and milliseconds since last PPS".

The correct model should probably be:
- `OurTime` = seconds (used for protocol timing decisions)
- `ref_time_ms` = the `millis()` timestamp corresponding to the start of that OurTime second
- `now_ms = millis()` = current milliseconds
- `ms_since_pps = now_ms - ref_time_ms` = milliseconds elapsed in current second (0-1000)

But currently `ref_time_ms` is being set to a seconds value, breaking this model.

