# Timing Architecture Considerations

## Overview
Discussion on 8-bit vs 16-bit timing for RTC-based task scheduler on ATtiny1616.

## Current Implementation (16-bit with DIV32)

### Configuration
- **Prescaler:** DIV32 (32.768kHz / 32 = 1024 Hz)
- **Tick period:** ~0.977ms per tick
- **Data type:** `uint16_t` for all time variables
- **Max schedulable delay:** ~64 seconds (65535 ticks)

### Timing Requirements
- **Debounce:** 50ms (51 ticks)
- **LED duration:** 100ms (102 ticks)
- **BPM range:** 40-155 BPM
  - 40 BPM: 1536 ticks per beat
  - 80 BPM: 768 ticks per beat
  - 155 BPM: 397 ticks per beat

### Issues
- **Not atomic:** 16-bit operations require 2 instructions on 8-bit AVR
- **Race conditions possible:** ISR could fire mid-operation during:
  - `currentTime = RTC.CNT` (2 instructions)
  - `timeDue = currentTime + delay` (multiple instructions)
  - Comparisons with wrap-around handling
- **Volatile overhead:** Need `volatile` qualifiers, compiler barriers

## 8-bit Timing with Slower Prescaler

### Option A: DIV256 (128 Hz)
- **Tick period:** 7.8ms per tick
- **Max schedulable delay:** 255 ticks = **2 seconds**
- **BPM calculations:**
  - 40 BPM: (60 × 128) / 40 = 192 ticks ✓
  - 80 BPM: (60 × 128) / 80 = 96 ticks ✓
  - 155 BPM: (60 × 128) / 155 = 49 ticks ✓
- **Debounce:** 50ms = ~6 ticks (47ms actual)
- **LED duration:** 100ms = ~13 ticks (102ms actual)

### Option B: DIV512 (64 Hz)
- **Tick period:** 15.6ms per tick
- **Max schedulable delay:** 255 ticks = **4 seconds**
- **BPM calculations:**
  - 40 BPM: 96 ticks ✓
  - 155 BPM: 25 ticks ✓
- **Debounce:** 50ms = ~3 ticks (47ms actual)
- **LED duration:** 100ms = ~6 ticks (94ms actual)

### With Current DIV32 (Not Viable)
- **Max schedulable delay:** 255 ticks = **249ms**
- **Minimum BPM:** ~241 BPM (only ultra-fast tempos work)
- **Conclusion:** Cannot support useful metronome range

## Atomicity Benefits of 8-bit

### Advantages
1. **All operations atomic:** Single instruction for all time reads/writes
2. **No race conditions:** ISR cannot corrupt time values
3. **Simpler code:** No need for complex synchronization
4. **No volatile complexity:** Atomic by hardware guarantee
5. **Easier to reason about:** Clearer correctness guarantees

### What Becomes Atomic
- `currentTime = RTC.CNT` → 1 LDS instruction
- `timeDue = currentTime + delay` → atomic read, atomic write
- All comparisons operate on 8-bit values

## Trade-offs Analysis

### Memory Savings (Minor)
- 16-bit → 8-bit saves ~18 bytes total:
  - 7 tasks × 2 bytes each = 14 bytes
  - `currentTime` variable = 2 bytes
  - `RTC.CMP` writes = 2 bytes
- On ATtiny1616 with 2KB RAM: **0.9% savings (negligible)**

### Timing Resolution
| Prescaler | Resolution | Human Perceptible? | Metronome Impact |
|-----------|------------|-------------------|------------------|
| DIV32 (current) | 0.977ms | No | Imperceptible |
| DIV256 | 7.8ms | No (at tempo level) | Acceptable |
| DIV512 | 15.6ms | Borderline | Possibly noticeable |

### Performance
- AVR is 8-bit native, but 16-bit operations are well-optimized
- No meaningful speed difference in practice
- Atomicity eliminates need for interrupt disabling

## Recommendations

### For This Metronome Application

**Recommended: DIV256 with 8-bit timing**

**Rationale:**
1. Human cannot perceive 7.8ms variance at metronome tempo
2. Atomic operations = robust, simple code
3. Full BPM range (40-155) supported with headroom
4. Timing precision adequate for musical purposes
5. Eliminates entire class of concurrency bugs

**Trade-off accepted:**
- Slightly coarser timing resolution (not perceptible)
- Debounce/LED timing rounded to nearest tick (acceptable)

### For Precision Timing Applications

**Keep 16-bit with DIV32 if:**
- Sub-millisecond precision required
- Audio sample-rate timing needed
- Memory not constrained enough to matter
- Willing to manage volatile/atomic complexity

## Implementation Impact

### Changes Required for 8-bit DIV256
1. Change `uint16_t` → `uint8_t` for:
   - `currentTime`
   - `Task.timeDue`
   - Function parameters/locals in scheduler
2. Update `RTC.CTRLA`: `RTC_PRESCALER_DIV32_gc` → `RTC_PRESCALER_DIV256_gc`
3. Update timing constants:
   - `DEBOUNCE_DELAY`: 50ms → 6 ticks
   - `LED_ON_DURATION`: 100ms → 13 ticks
   - BPM formula: `(60 × 128) / bpm` instead of `(60 × 1024) / bpm`
4. Update comments about tick rates
5. Remove `volatile` qualifiers (no longer needed for atomicity)

### Code Robustness Improvements
- No need for critical sections around time reads
- ISRs can safely schedule without corruption risk
- Simpler mental model for developers
- Fewer subtle timing bugs possible

## Conclusion

For a metronome application, **8-bit timing with DIV256 prescaler offers better engineering trade-offs** than 16-bit timing:
- Atomic operations eliminate concurrency complexity
- Timing resolution more than adequate for human perception
- Simpler, more maintainable code
- Negligible memory savings but significant robustness gain

The 16-bit approach is only preferable if sub-millisecond precision is genuinely required, which it is not for tempo-based metronome applications.
