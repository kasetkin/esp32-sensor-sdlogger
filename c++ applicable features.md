# Applicable C++ Features (C++17 – C++26)

**Project:** ESP32-C6 firmware — ESP-IDF v6.0, GCC 15.2.0, ~8500 lines of C++
**Prerequisite:** Add to `main/CMakeLists.txt`:
```cmake
set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD 23)
```

---

## C++17 — 12 applicable features

### 1. Inline variables
**Location:** `modules/TinyGPSPlus/src/unicore.h:85`
```cpp
// Before — ODR violation: each TU that includes unicore.h gets its own ~1 KB copy
const std::uint32_t aulCrcTable[256] = { 0x00000000UL, ... };

// After — linker merges all copies into one
inline constexpr std::uint32_t aulCrcTable[256] = { 0x00000000UL, ... };
```

---

### 2. Structured bindings
**Location:** `modules/TinyGPSPlus/src/unicore.cpp:81–83`
```cpp
// Before
const auto preparedPair = splitAndPrepareString(str);
const auto &leapSecsStr = preparedPair.first;
const auto &solStatusStr = preparedPair.second;

// After
const auto [leapSecsStr, solStatusStr] = splitAndPrepareString(str);
```

---

### 3. `if`/`switch` with initializer
**Location 1:** `modules/TinyGPSPlus/src/unicore.cpp:69–74`
```cpp
// Before
const auto delimiterPos = cppStr.find(';');
if (delimiterPos != std::string::npos) {

// After — scopes delimiterPos to the if block
if (const auto delimiterPos = cppStr.find(';'); delimiterPos != std::string::npos) {
```
**Location 2:** `main/sensorstask.cpp:14` — same pattern for `dotPos`.

---

### 4. `[[fallthrough]]`
**Location:** `modules/TinyGPSPlus/src/TinyGPS++.cpp:83` and `:103`
```cpp
// Before — GCC does NOT recognize comment form; emits -Wimplicit-fallthrough
// [[fallthrough]];

// After
[[fallthrough]];
```

---

### 5. `[[nodiscard]]`
**Locations:**
- `main/sdcard.h:18,21,28,30` — `format()`, `unmount()`, `writeFile()`, `appendFile()` all return `esp_err_t` that is silently discarded
- `main/common_utils.h:13` — `registerWakeupTimer()` — ignoring leaves device unable to wake from deep sleep
- `main/gpstask.h:73` — `configureUart()` — ignoring silently corrupts GPS data

```cpp
// Before
esp_err_t appendFile(const std::string &path, const char *data);

// After
[[nodiscard("ignoring drops log data")]] esp_err_t appendFile(const std::string &path, const char *data);
```

---

### 6. `std::byte`
**Location:** `main/gpstask.cpp:688–789` — `appendRaw` + `emulateQstarzBinary`
```cpp
// Before — std::string misused as raw binary storage
std::string buf;
template<typename T>
void appendRaw(std::string &buf, const T &data) {
    buf.append(reinterpret_cast<const char *>(&data), sizeof(T));
}

// After — explicit raw-byte semantics
std::vector<std::byte> buf;
buf.reserve(64);
template<typename T>
void appendRaw(std::vector<std::byte> &buf, const T &data) {
    const auto *p = reinterpret_cast<const std::byte *>(&data);
    buf.insert(buf.end(), p, p + sizeof(T));
}
```

---

### 7. `std::optional`
**Location 1:** `main/sensorstask.h` — `SensorsValues` struct uses `-1` for ints and `NaN` for floats as "absent" sentinels
```cpp
// Before — two different "no value" conventions in one struct
struct SensorsValues {
    int batteryVoltageMilliV = -1;
    int batteryPercent = -1;
    float envTemperature = std::numeric_limits<float>::quiet_NaN();
    float envHumidity = std::numeric_limits<float>::quiet_NaN();
    float barometricPressure = std::numeric_limits<float>::quiet_NaN();
};

// After
struct SensorsValues {
    std::optional<int> batteryVoltageMilliV;
    std::optional<int> batteryPercent;
    std::optional<float> envTemperature;
    std::optional<float> envHumidity;
    std::optional<float> barometricPressure;
};
```
**Location 2:** `main/bleservertask.h` — `m_batteryLevel = -1.0f`, `m_envTemperature = -275.0f`, `m_envHumidity = -1.0f` — three different magic sentinel conventions → `std::optional<float>`.

---

### 8. `std::string_view`
**Locations:** `main/loggertask.h`, `main/bleservertask.h`, `main/gpstask.h` — ~8 functions accepting `const std::string &` that only read the string
```cpp
// Before
void addNmeaLog(const std::string &nmeaMessage);
void setGpsLog(const std::string &log);

// After
void addNmeaLog(std::string_view nmeaMessage);
void setGpsLog(std::string_view log);
```
Also: `main/loggertask.cpp:13–15` — `static const std::string ownerId/ownerShortName/ownerFullName` → `static constexpr std::string_view`.

---

### 9. `std::to_chars` / `std::from_chars`
**Location:** `main/gpstask.cpp` — `printGpsGeoInfo`/`printPppGeoInfo` call `std::to_string()` ~10× per GPS fix; `atoi`/`atol` in parsing hot path
```cpp
// Before — heap allocates per call
message += std::string(";SATS;") + std::to_string(p.satellites.size());

// After — zero allocation, no locale overhead
char buf[32];
auto [ptr, ec] = std::to_chars(buf, buf + sizeof(buf), p.satellites.size());
message.append(buf, ptr);

// Before (parsing)
gpsInfo.fixType = atoi(gsafixtype.value());

// After
int32_t val;
std::from_chars(str, str + len, val);
```

---

### 10. `std::clamp`
**Location 1:** `main/sensorstask.cpp:271`
```cpp
// Before
return std::max<double>(0.0f, std::min<double>(100.0, value));

// After
return std::clamp(value, 0.0, 100.0);
```
**Location 2:** `main/bleservertask.cpp:806`
```cpp
// Before
const uint8_t level = static_cast<uint8_t>(std::min(100.0f, std::max(0.0f, std::round(m_batteryLevel))));

// After
const uint8_t level = static_cast<uint8_t>(std::clamp(std::round(m_batteryLevel), 0.0f, 100.0f));
```

---

### 11. Non-member `std::size`
**Location:** `modules/TinyGPSPlus/src/TinyGPS++.cpp:397`
```cpp
// Before — magic literal
return directions[direction % 16];

// After — auto-tracks array size
return directions[direction % std::size(directions)];
```

---

### 12. `std::pmr` (polymorphic memory resources)
**Location:** `main/bleservertask.h` — `m_nmeaStream` and `m_logStream` fully populated then drained every 1 Hz BLE send cycle
```cpp
// Before
std::vector<std::string> m_nmeaStream;
std::vector<std::string> m_logStream;

// After — eliminates repeated heap malloc/free per send cycle
std::array<std::byte, 4096> m_streamBuf;
std::pmr::monotonic_buffer_resource m_streamPool{m_streamBuf.data(), m_streamBuf.size()};
std::pmr::vector<std::pmr::string> m_nmeaStream{&m_streamPool};
std::pmr::vector<std::pmr::string> m_logStream{&m_streamPool};
// call m_streamPool.release() at end of sendAllData()
```

---

## C++20 — 14 applicable features

### 13. Concepts
**Location:** `main/gpstask.cpp:687`
```cpp
// Before
template<typename T>
void appendRaw(std::vector<std::byte> &buf, const T &data) { ... }

// After — constraint enforced at call site with a clear error message
template<typename T>
    requires std::is_trivially_copyable_v<T>
void appendRaw(std::vector<std::byte> &buf, const T &data) { ... }
```

---

### 14. Ranges + Range views
**Location 1:** `main/gpstask.cpp:304–332` — multi-condition OR chain for GPS lock quality
```cpp
// Before
bool hasLock = info.fixType == 3 || info.fixType == 2 || info.fixType == 1;

// After
constexpr std::array validFix{1, 2, 3};
bool hasLock = std::ranges::contains(validFix, info.fixType);
```
**Location 2:** `modules/TinyGPSPlus/src/unicore.cpp:55–63` — `prepareString` transform
```cpp
// Before
std::string result;
std::transform(str.begin(), str.end(), std::back_inserter(result),
               [](char c) { return std::toupper(c); });

// After
auto result = str | std::views::transform([](char c) static { return std::toupper(c); })
                  | std::ranges::to<std::string>();
```
**Location 3:** `main/bleservertask.cpp:74–81` — hex-char filter loop → `std::views::filter`
**Location 4:** `main/main.cpp:166–168` — CRLF trim loop → `std::views::reverse | std::views::drop_while`

---

### 15. Spaceship operator `<=>`
**Location 1:** `main/gpstask.h:24–36` — `SatelliteInfo` has manual `operator==` and 4-field `operator<`
```cpp
// Before
bool operator==(const SatelliteInfo &other) const { ... }
bool operator<(const SatelliteInfo &other) const { ... }

// After
auto operator<=>(const SatelliteInfo &) const = default;
```
**Location 2:** `modules/TinyGPSPlus/src/TinyGPS++.h:43–51` — `RawDegrees` manual comparators → `= default`.

---

### 16. `[[likely]]` / `[[unlikely]]`
**Location 1:** `modules/TinyGPSPlus/src/TinyGPS++.cpp:114` — character-by-character NMEA decode hot path
```cpp
if (c == ',') [[likely]] { ... }
```
**Location 2:** `main/gpstask.cpp:254` — GPS fix present check
**Location 3:** `main/sensorstask.cpp:239` — ADC error check
```cpp
if (err != ESP_OK) [[unlikely]] { ... }
```

---

### 17. Designated initializers
**Location 1:** `main/gpstask.cpp:401` — `struct tm t` with manual field assignments
```cpp
// Before — tm_wday and tm_yday implicitly uninitialized
struct tm t;
t.tm_sec = ...; t.tm_min = ...; ...

// After — zero-initializes all fields including tm_wday, tm_yday
struct tm t = {
    .tm_sec  = static_cast<int>(p.time.second()),
    .tm_min  = static_cast<int>(p.time.minute()),
    .tm_hour = static_cast<int>(p.time.hour()),
    .tm_mday = static_cast<int>(p.date.day()),
    .tm_mon  = static_cast<int>(p.date.month()) - 1,
    .tm_year = static_cast<int>(p.date.year()) - 1900,
    .tm_isdst = 0,
};
```
**Location 2:** `main/loggertask.cpp:166` — same pattern for `struct tm gmTime`.

---

### 18. `consteval`
**Location 1:** `main/bleservertask.cpp:42` — `buildBleUuid16` is a compile-time-only UUID builder
```cpp
// Before
constexpr ble_uuid16_t buildBleUuid16(uint16_t value) { ... }

// After — compile-time-only; error if called at runtime
consteval ble_uuid16_t buildBleUuid16(uint16_t value) { ... }
```
**Location 2:** `main/sensorstask.h:52` — `voltageDividerCoefficient` computed from compile-time resistor values.

---

### 19. `constinit`
**Location 1:** `main/bleservertask.cpp:19` — file-scope variable initialized from a compile-time expression
```cpp
// Before
uint8_t own_addr_type = 0;

// After — documents and enforces compile-time initialization
constinit uint8_t own_addr_type = 0;
```
**Location 2:** `main/loggertask.cpp:13–15` — `ownerId`/`ownerShortName`/`ownerFullName` → `constinit static constexpr std::string_view`.
**Location 3:** `main/main.cpp:18` — similar static initializer.

---

### 20. `using enum`
**Location 1:** `modules/TinyGPSPlus/src/unicore.cpp:238` — switch on `PositionVelocityType` with 21× repeated `PositionVelocityType::` prefix
```cpp
// Before
case PositionVelocityType::NONE: ...
case PositionVelocityType::FIXEDPOS: ...

// After
using enum PositionVelocityType;
case NONE: ...
case FIXEDPOS: ...
```
**Location 2:** `modules/TinyGPSPlus/src/unicore.cpp:258` — same enum, second switch.
**Location 3:** `main/gpstask.cpp:304` — fix-quality enum switch.

---

### 21. `[[nodiscard]]` with reason
**Locations:** Same as C++17 `[[nodiscard]]` above, but with the message argument:
```cpp
[[nodiscard("ignoring drops log data silently")]] esp_err_t appendFile(...);
[[nodiscard("ignoring leaves device unable to wake from deep sleep")]] esp_err_t registerWakeupTimer(...);
[[nodiscard("UART misconfiguration silently corrupts GPS data")]] bool configureUart();
```

---

### 22. `std::format`
**Location 1:** `main/gpstask.cpp:514,570` — `snprintf` for GPS date/time formatting
```cpp
// Before
snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02dZ",
         year, month, day, hour, min, sec);

// After
auto s = std::format("{:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}Z",
                     year, month, day, hour, min, sec);
```
**Location 2:** `main/sensorstask.cpp:23` — sensor value formatting.

---

### 23. `std::span`
**Location 1:** `main/bleservertask.h:121` — `bleTx(void* data, size_t len, ...)` → type-safe
```cpp
// Before
void bleTx(void *data, size_t len, uint16_t connHandle);

// After
void bleTx(std::span<const std::byte> data, uint16_t connHandle);
```
**Location 2:** `main/sdcard.h:28–30` — `writeFile`/`appendFile` with raw `const char*` + implicit length.

---

### 24. `std::numbers::*`
**Location:** `modules/TinyGPSPlus/src/TinyGPS++.cpp:334,340,390` and `main/gpstask.cpp:497`
```cpp
// Before
const double PI = 3.14159265358979323846;  // or M_PI macro

// After
#include <numbers>
// use std::numbers::pi directly
```

---

### 25. Range views (`std::views::chunk`, `std::views::filter`, etc.)
**Location:** `main/bleservertask.cpp:750–791` — MTU chunking loop
```cpp
// Before — manual division + remainder
while (!line.empty()) {
    auto chunk = line.substr(0, maximumPayloadSize);
    line = line.substr(chunk.size());
    bleTx(...);
}

// After
for (auto chunk : line | std::views::chunk(maximumPayloadSize)) {
    bleTx(std::span(chunk), connHandle);
}
```

---

### 26. `starts_with` / `ends_with`
**Location:** `modules/TinyGPSPlus/src/unicore.cpp:168`
```cpp
// Before
cppStr[0] == '"' && cppStr[cppStr.size() - 1] == '"'

// After
cppStr.starts_with('"') && cppStr.ends_with('"')
```

---

## C++23 — 10 applicable features

### 27. `static operator()`
**Location:** `main/bleservertask.cpp:188,195,202` — non-capturing FreeRTOS task trampolines
```cpp
// Before — hidden this pointer passed unnecessarily
auto task = [](void *arg) { ... };

// After — no hidden this; directly convertible to void(*)(void*)
auto task = [](void *arg) static { ... };
```

---

### 28. `#warning`
**Location 1:** `main/gpstask.cpp:71` — TODO comment that silently goes unnoticed
```cpp
// Before
// TODO: handle timeout properly

// After — emits a compiler warning
#warning "TODO: handle timeout properly"
```
**Location 2:** `main/gpstask.cpp:414`
**Location 3:** `main/sensorstask.cpp:285`

---

### 29. `[[assume(expr)]]`
**Location 1:** `main/bleservertask.cpp:750` — before MTU chunk loop
```cpp
[[assume(maximumPayloadSize > 0)]];
for (auto chunk : line | std::views::chunk(maximumPayloadSize)) { ... }
```
**Location 2:** `main/sensorstask.cpp:238` — before ADC averaging loop
```cpp
[[assume(sampleCount > 0)]];
```
**Location 3:** `main/gpstask.cpp:768` — packet size invariant in binary protocol builder.

---

### 30. `constexpr std::string` / `std::string_view`
**Location:** `main/loggertask.cpp:13–15`
```cpp
// Before
static const std::string ownerId = "K6AKS";
static const std::string ownerShortName = "Alex";
static const std::string ownerFullName = "Alex Kasatkin";

// After — no heap allocation, pure compile-time constant
static constexpr std::string_view ownerId = "K6AKS";
static constexpr std::string_view ownerShortName = "Alex";
static constexpr std::string_view ownerFullName = "Alex Kasatkin";
```

---

### 31. `std::expected<T, E>`
**Location 1:** `main/sensorstask.h:76` / `sensorstask.cpp:221` — `readBatteryVoltageMilliV()` returns `-1` on error
```cpp
// Before
int readBatteryVoltageMilliV();  // -1 = error

// After
std::expected<int, esp_err_t> readBatteryVoltageMilliV();
```
**Location 2:** `main/gpstask.h:100` — `sendStringAndWait()` returns bool but carries no error detail
```cpp
// Before
bool sendStringAndWait(std::string_view cmd, ...);

// After
std::expected<std::string, esp_err_t> sendStringAndWait(std::string_view cmd, ...);
```

---

### 32. `std::flat_set`
**Location:** `main/gpstask.h:45,130` — `std::set<SatelliteInfo>` used for GPS satellite tracking
```cpp
// Before — red-black tree, pointer-chasing, heap fragmentation
std::set<SatelliteInfo> satellites;
std::set<SatelliteInfo> m_pendingSatellites;

// After — sorted contiguous array, cache-friendly, no tree overhead
std::flat_set<SatelliteInfo> satellites;
std::flat_set<SatelliteInfo> m_pendingSatellites;
```

---

### 33. New range views (`std::views::chunk`, `std::views::enumerate`, etc.)
**Location 1:** `main/bleservertask.cpp:770` — MTU chunking (see also feature #25)
```cpp
for (auto chunk : line | std::views::chunk(maximumPayloadSize)) { ... }
```
**Location 2:** `main/gpstask.cpp:151` — indexed iteration over satellite array
```cpp
// Before
for (size_t i = 0; i < gsaSat.size(); ++i) { ... gsaSat[i] ... }

// After
for (auto [i, sat] : std::views::enumerate(gsaSat)) { ... }
```

---

### 34. `std::ranges::contains`
**Location:** `main/gpstask.cpp:304–332` — multi-OR quality check
```cpp
// Before
bool hasLock = (info.fixType == 3 || info.fixType == 2 || info.fixType == 0);

// After
constexpr std::array validFix{0, 2, 3};
bool hasLock = std::ranges::contains(validFix, info.fixType);
```

---

### 35. `std::to_underlying`
**Location:** `main/gpstask.cpp:729`
```cpp
// Before
static_cast<uint8_t>(p.quality)

// After
static_cast<uint8_t>(std::to_underlying(p.quality))
```

---

### 36. `std::unreachable()`
**Location:** `main/bleservertask.cpp:506–507` — default arm of an exhaustive switch
```cpp
// Before
default:
    assert(0);
    break;

// After
default:
    std::unreachable();
```

---

## C++26 — 3 applicable features (available now in GCC 15.2.0)

### 37. `= delete("reason")`
**Location 1:** All 5 task class headers (`sdcard.h`, `gpstask.h`, `sensorstask.h`, `loggertask.h`, `bleservertask.h`) — implicitly-deleted copy gives a cryptic cascading error
```cpp
// Before — error message is a cascade of deleted-member errors
class SdCard { ... };  // copy implicitly deleted via non-copyable members

// After — one clear message
class SdCard {
public:
    SdCard(const SdCard &) = delete("SdCard owns an SPI bus handle — copying aliases hardware resources");
    SdCard &operator=(const SdCard &) = delete("SdCard owns an SPI bus handle — copying aliases hardware resources");
```
**Location 2:** `main/common_utils.h:13` — `registerWakeupTimer(uint32_t)` — silent sign wrap for negative int
```cpp
esp_err_t registerWakeupTimer(uint32_t wakeupMicrosec);
esp_err_t registerWakeupTimer(int) = delete("duration must be uint32_t microseconds — negative values silently wrap to enormous sleep");
```

---

### 38. Bounds safety for containers (`_GLIBCXX_ASSERTIONS`)
**Action:** Add to `main/CMakeLists.txt` for debug builds:
```cmake
target_compile_definitions(${COMPONENT_LIB} PRIVATE $<$<CONFIG:Debug>:_GLIBCXX_ASSERTIONS>)
```
**Why:** `main/gpstask.cpp:779–784` — `emulateQstarzBinary()` accesses `packets[2][16]`–`packets[2][19]` on `std::string` members relying solely on `assert(buf.size() == 64)`, which is stripped in release. Hardened mode turns silent UB into a terminated diagnostic in debug builds. Zero source changes required.

---

### 39. `std::string::contains`
**Location:** `main/gpstask.cpp:65,133`
```cpp
// Before
if (reply.find("UM980") != std::string::npos) { ... }
if (reply.find("system is rebooting") != std::string::npos) { ... }

// After
if (reply.contains("UM980")) { ... }
if (reply.contains("system is rebooting")) { ... }
```

---

## Summary

| Standard | Applicable | Total analyzed |
|----------|-----------|---------------|
| C++17    | 12        | 43            |
| C++20    | 14        | 41            |
| C++23    | 10        | 30            |
| C++26    | 3         | 23            |
| **Total**| **39**    | **137**       |

### Quick wins (minimal risk, high clarity)
1. `[[fallthrough]]` — uncomment two lines in `TinyGPS++.cpp`
2. `std::string::contains` — two replacements in `gpstask.cpp`
3. `std::clamp` — two replacements
4. `std::numbers::pi` — replace `M_PI` macro
5. `_GLIBCXX_ASSERTIONS` — one CMakeLists.txt line

### Highest impact (correctness / safety)
1. `[[nodiscard]]` with reason — silent SD write failures, deep-sleep misconfiguration
2. `std::to_chars`/`from_chars` — eliminates ~10 heap allocations/second in GPS hot path
3. `std::pmr` — eliminates repeated malloc/free in 1 Hz BLE send cycle
4. `std::optional` — unifies mixed sentinel conventions across `SensorsValues`
5. `inline constexpr aulCrcTable` — fixes active ODR violation in `unicore.h`
