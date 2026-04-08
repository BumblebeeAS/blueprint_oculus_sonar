# Oculus Sonar Data Pipeline

Deep-dive into how raw sonar data flows from hardware through the driver into ROS messages and image processing. For package usage, see [README.md](README.md).

## Overview

```
Oculus M750d hardware
        |
        v
oculus_driver::SonarDriver        -- parses raw stream into PingMessage
        |
        v
OculusSonarNode::publishPing()    -- converts to ROS msg, publishes all topics
        |
        +---> sonar_viewer_.publishFan()         -- cartesian conic image
        +---> sonar_viewer_.publishRaw()         -- polar bin-beam image
        +---> sonar_viewer_.publishPointCloud()  -- 3D point cloud
```

## 1. Hardware to Driver (`oculus_driver`)

The external `oculus_driver` library (fetched via CMake `FetchContent`) handles TCP communication with the sonar. `SonarDriver` opens a persistent TCP connection, receives raw byte streams, and parses them into `PingMessage` objects. The node registers callbacks at startup:

```cpp
sonar_driver_->add_ping_callback(&OculusSonarNode::publishPing, this);
sonar_driver_->add_status_callback(&OculusSonarNode::publishStatus, this);
```

Each time the sonar fires a ping, the driver invokes `publishPing()` with a `PingMessage::ConstPtr`.

## 2. Ping Message Construction (`OculusMessage.h`)

Source: [`oculus_driver/include/oculus_driver/OculusMessage.h`](https://github.com/ENSTABretagneRobotics/oculus_driver/blob/master/include/oculus_driver/OculusMessage.h)

The driver constructs ping messages through a layered class hierarchy that sits on top of packed C structs defined in `Oculus.h`.

### Raw Hardware Structs (`Oculus.h`)

The sonar sends TCP packets whose payload maps directly onto these packed structs:

**`OculusSimplePingResult`** (message version 1):
```c
typedef struct {
    OculusSimpleFireMessage fireMessage;  // echo of the fire config that triggered this ping
    uint32_t pingId;            // incrementing ping counter
    uint32_t status;
    double   frequency;         // acoustic frequency (Hz)
    double   temperature;       // external temperature (deg C)
    double   pressure;          // external pressure (bar)
    double   speeedOfSoundUsed; // actual speed of sound used (m/s)
    uint32_t pingStartTime;     // sonar internal clock (microseconds, wraps ~1h)
    uint8_t  dataSize;          // enum: 8-bit, 16-bit, 24-bit, or 32-bit
    double   rangeResolution;   // meters per range bin
    uint16_t nRanges;           // number of range lines (rows)
    uint16_t nBeams;            // number of bearings (columns)
    uint32_t imageOffset;       // byte offset of image data from start of message
    uint32_t imageSize;         // byte size of image data
    uint32_t messageSize;       // total network message size in bytes
    // Followed in memory by: int16_t bearings[nBeams] then image data
} OculusSimplePingResult;
```

**`OculusSimplePingResult2`** (message version 2) adds heading/pitch/roll, uses `OculusSimpleFireMessage2`, and has `pingStartTime` as `double` (seconds with microsecond resolution) instead of `uint32_t`.

**`OculusSimpleFireMessage`** (embedded in the ping result):
```c
typedef struct {
    OculusMessageHeader head;
    uint8_t  masterMode;       // 0=flexi, 1=low freq (1.2MHz), 2=high freq (2.1MHz)
    uint8_t  pingRate;         // 0=10Hz, 1=15Hz, 2=40Hz, 3=5Hz, 4=2Hz, 5=standby
    uint8_t  networkSpeed;
    uint8_t  gammaCorrection;
    uint8_t  flags;            // bit 0: range as meters, bit 1: 16-bit data,
                               // bit 2: send gains, bit 3: simple ping,
                               // bit 4: gain assist, bit 6: 512 beams
    double   range;            // range in meters or percent (depends on flags bit 0)
    double   gainPercent;
    double   speedOfSound;     // m/s, 0 = use internal calculation
    double   salinity;         // ppt, 0 = fresh water
} OculusSimpleFireMessage;
```

### Driver Class Hierarchy

```
Message                    -- owns raw byte buffer + OculusMessageHeader
    |
PingWrapper (abstract)     -- interface for ping metadata (virtual methods)
    |
    +-- PingWrapper1       -- wraps OculusSimplePingResult  (msg version 1)
    +-- PingWrapper2       -- wraps OculusSimplePingResult2 (msg version 2)
    |
PingMessage                -- public API, auto-selects wrapper by version
```

**`Message`** stores the complete raw TCP payload as `std::vector<uint8_t> data_` and casts the front of that buffer to `OculusMessageHeader`. Only `SonarClient` and `FileReader` can modify it (friend classes).

**`PingWrapper1`/`PingWrapper2`** reinterpret-cast the same `data_` buffer to `OculusSimplePingResult` / `OculusSimplePingResult2` and expose typed accessors:

```cpp
// PingWrapper1::metadata() reinterprets the raw buffer
const OculusSimplePingResult& metadata() const {
    return *reinterpret_cast<const OculusSimplePingResult*>(msg_->data().data());
}

// Bearing data sits right after the struct in memory
const int16_t* bearing_data() const {
    return (const int16_t*)(data().data() + sizeof(OculusSimplePingResult));
}

// Image data starts at the offset specified in the struct
const uint8_t* ping_data() const {
    return data().data() + metadata().imageOffset;
}
```

**`PingMessage`** auto-selects the correct wrapper based on `message_version()`:
```cpp
static PingWrapper::Ptr make_ping_wrapper(const Message::ConstPtr& msg) {
    if (msg->message_version() == 2)
        return PingWrapper2::Create(msg);
    else
        return PingWrapper1::Create(msg);
}
```

### Key Offset Calculations

The `data()` vector (exposed to ROS as `ping_data`) contains the full message:

```
[ OculusSimplePingResult header | int16_t bearings[nBeams] | image rows... ]
|<------- imageOffset bytes -------------------------------->|
```

- **`imageOffset`**: byte offset from start of message to first image row (stored in the struct itself)
- **`ping_data_offset()`**: same as `imageOffset`, computed as pointer difference: `ping_data() - data().data()`
- **`bearing_data_offset()`**: `sizeof(OculusSimplePingResult)` — bearings immediately follow the struct
- **`step()`**: bytes per image row, computed as: `(has_gains ? 4 : 0) + bearing_count * sample_size`
- **`has_gains()`**: version 1 checks `fireMessage.flags & 0x4`; version 2 infers from `imageSize > sample_size * nBeams * nRanges`
- **`sample_size()`**: maps `dataSize` enum → bytes (0→1, 1→2, 2→3, 3→4), with fallback deduction from message size if the enum is invalid

## 3. Driver to ROS Message (`conversions.hpp`)

`oculus::toMsg()` copies fields from the driver's `PingMessage` into `oculus_interfaces::msg::Ping`:

```cpp
msg.n_ranges    = ping->range_count();    // rows in the image
msg.n_beams     = ping->bearing_count();  // columns in the image
msg.step        = ping->step();           // bytes per row (gain + pixels)
msg.sample_size = ping->sample_size();    // bytes per pixel (1, 2, or 4)
msg.has_gains   = ping->has_gains();      // whether rows start with 4-byte gain
msg.bearings.assign(ping->bearing_data(), ...);  // int16[], 100ths of degree
msg.ping_data   = ping->data();           // full raw buffer (headers + image)
```

The `ping_data` field contains the **entire raw buffer** from the driver, not just image pixels. Headers and metadata are prepended before the actual image rows.

## 4. Ping Data Memory Layout

The `ping_data` byte vector has this structure:

```
[ ... headers/metadata ... | row 0 | row 1 | ... | row (n_ranges-1) ]
                           ^
                           image_start = ping_data.size() - (n_ranges * step)
```

Each row is `step` bytes wide and laid out as:

```
|  gain (4 bytes, if has_gains)  |  pixel_0  |  pixel_1  |  ...  |  pixel_(n_beams-1)  |
|<------ gain_offset = 4 ------>|<---------- n_beams * sample_size ------------------>|
|<------------------------------ step bytes ----------------------------------------->|
```

- **Gain**: Little-endian `uint32`. Normalize by dividing the row intensity by `sqrt(gain)`.
- **Pixels**: Little-endian, sized by `sample_size` (1 = `uint8`, 2 = `uint16`, 4 = `uint32`).
- **Bearings**: `int16[]` array of angles in hundredths of a degree. Non-uniform spacing. Convert: `radians = bearing * 0.01 * PI / 180`.

## 5. `pingToIntensity()` -- Extracting the Image

This is the core function that turns raw `ping_data` into an OpenCV `CV_8UC1` intensity matrix (`sonar_viewer.cpp:158`).

**Step-by-step:**

1. **Find image start offset**:
   ```cpp
   data_offset = ping_data.size() - (n_ranges * step);
   ```
   This skips over any prepended headers/metadata in the raw buffer.

2. **For each range row `r`**:
   ```cpp
   row_start = data_offset + r * step;
   ```

3. **Read per-row gain** (first 4 bytes at `row_start`, little-endian `uint32`):
   ```cpp
   gain = ping_data[row_start] | (ping_data[row_start+1] << 8) | ...;
   gain_norm = 1.0 / sqrt(gain);
   ```

4. **Read each pixel** at offset `row_start + gain_offset + b * sample_size`:
   - Decode as `uint8`, `uint16`, or `uint32` depending on `sample_size`
   - Apply gain: `normalized = raw * gain_norm`
   - Scale to 0-255: `output = min(255, normalized / max_val * 255)`

## 6. `publishRaw()` -- Polar Image

Converts the intensity matrix from sonar-native (range x bearing) coordinates into a proper polar image using OpenCV `remap()`. The remap tables are cached and only regenerated when `n_beams`, `n_ranges`, or image dimensions change.

Bearing angles are converted to radians and used to compute (x, y) positions for each pixel:
```cpp
bearing_rad = bearings[i] * 0.01 * PI / 180.0;
x = range_resolution * (rows - i);
y = range_resolution * (j - cols/2.0 + 0.5);
```

## 7. `publishFan()` -- Cartesian Conic Image

Maps sonar data from polar to Cartesian coordinates to produce a sector/fan visualization. Uses the frequency-dependent bearing aperture:
- Low frequency (mode 1): 65 degrees
- High frequency (mode 2): 40 degrees

For each output pixel, computes range and bearing from origin, then remaps from the transposed sonar matrix using cubic interpolation.

## 8. `publishPointCloud()` -- 3D Point Cloud

Converts every (range, bearing) cell into a 3D point in the sonar frame (X=forward, Y=left, Z=up):

```cpp
range   = (r + 1) * range_resolution;
bearing = bearings[b] * 0.01 * PI / 180.0;
x = range * cos(bearing);
y = -range * sin(bearing);
z = 0.0;  // horizontal sonar plane
intensity = pixel_value / 255.0;
```

## 9. Downstream Standalone Nodes

In addition to the main `OculusSonarNode` which publishes everything, there are standalone nodes that subscribe to `oculus/ping` and run independently:

- **`sonar_raw_image_node`** -- subscribes to `oculus/ping`, publishes `oculus/not_polar` (remapped Cartesian image)
- **`sonar_pointcloud_node`** -- subscribes to `oculus/ping`, publishes `oculus/pointcloud2`

These use the same `SonarViewer` functions (`pingToIntensity`, `pingToImageConversion`) internally.
