# USRP tools
This repository contains some of the UHD library's host utilities/examples that I separated from the [UHD host](https://https://github.com/EttusResearch/uhd/tree/master/host) libraries. Also it's a way for me to experiment with [Meson](https://mesonbuild.com/) :).

## Sources
The `src` directory contains the following executables:

-  `query_gpsdo_sensors.cc`: To detect whether your USRP has a GPSDO or not
-  `sync_to_gps.cc`: To test that you can align GPSDO time to USRP's internal clock.
-  `rx_timed_samples_gpsdo.cc`: To test that you can obtain timed samples aligned to internal or GPS time.
-  `rx_timed_samples_to_file.cc`: To store raw IQ data with timing information.

## Building

### Prerequisites
You'll need to install the `udh` and `boost-program-options` libraries. Also you will need `meson` and a compiler toolchain. For example on **Debian/Ubuntu** based systems these should be achieved with:

```bash
sudo apt install meson build-essential libboost-dev libboost-program-options-dev libuhd-dev
```

### Compiling
On the root directory run:
```bash
meson setup build
```

Then navigate to the `build` directory and run:
```bash
cd build && meson compile
```
and that should do the trick!