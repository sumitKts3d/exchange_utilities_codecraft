# exchange_utilities_codecraft — HOOPS Exchange plugins

Source code and build files of four converters built on the HOOPS Exchange public API with CodeCraft:

| Plugin | What it does | Needs |
| --- | --- | --- |
| [`QIF_Reader`](QIF_Reader/README.md) | QIF 3.0 → PRC (exact B-rep, appearance, structure, graphical and semantic PMI) | HOOPS Exchange |
| [`QIF_Writer`](QIF_Writer/README.md) | any CAD file / PRC → QIF 3.0 | HOOPS Exchange |
| [`USD_Reader`](USD_Reader/README.md) | OpenUSD (`.usd/.usda/.usdc/.usdz`) → PRC | HOOPS Exchange, OpenUSD |
| [`USD_Writer`](USD_Writer/README.md) | any CAD file / PRC → OpenUSD | HOOPS Exchange, OpenUSD |

`usd_common/` holds the OpenUSD wiring and the conventions shared by the two USD plugins.

The repository contains source only: no HOOPS Exchange files (headers, libraries, DLLs, license key), no OpenUSD and no
build output. The plugins are built like the samples shipped with HOOPS Exchange (same `find_package(Exchange)`, same
common options, same `SampleTools` helper libraries, same test labels).

## Where to put it

Clone this repository as the `tools/plugins` folder of a HOOPS Exchange installation:

```
<HOOPS Exchange>/
├── bin/
│   └── usd/            OpenUSD binary package (only for USD_Reader / USD_Writer)
├── include/            (hoops_license.h with a valid license key)
├── samples/
└── tools/
    └── plugins/        <- this repository
        ├── CMakeLists.txt
        ├── QIF_Reader/  QIF_Writer/  USD_Reader/  USD_Writer/  usd_common/
```

```
cd <HOOPS Exchange>/tools
git clone https://github.com/sumitKts3d/exchange_utilities_codecraft plugins
```

The HOOPS Exchange folder is found two levels above the repository. Another layout works with
`-DA3DLIBS_ROOT=<HOOPS Exchange folder>` (or the `A3DLIBS_ROOT` environment variable).

## Requirements

- HOOPS Exchange 2026.8.0 with a valid key in `include/hoops_license.h` (a missing or placeholder key builds but stops at startup).
- CMake 3.21 or newer and Visual Studio with the C++ workload (the plugins are developed and tested on Windows x64).
- OpenUSD binary package (only for the USD plugins), tested with NVIDIA's `usd.py312.windows-x86_64.usdview.release-v25.08`:
  1. `<HOOPS Exchange>/bin/usd` if that folder exists (it must contain `include/pxr/pxr.h` and `cmake/pxrTargets.cmake`);
  2. otherwise the folder given by `-DUSD_ROOT=<OpenUSD folder>` or the `USD_ROOT` environment variable.

  If OpenUSD is found in neither place, configuration stops with an explanation. Build `Release` (the package ships release binaries).

## Build

```bat
cd <HOOPS Exchange>\tools\plugins
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Executables are written to `build\bin\Release`. Useful variants:

```bat
:: QIF plugins only (no OpenUSD needed)
cmake -S . -B build -DHOOPS_PLUGINS_BUILD_USD=OFF

:: HOOPS Exchange or OpenUSD in another place
cmake -S . -B build -DA3DLIBS_ROOT=D:\HOOPS_Exchange_2026.8.0 -DUSD_ROOT=D:\usd

:: one plugin (QIF_Writer / USD_Writer tests also need the matching reader)
cmake --build build --config Release --target QIF_Reader
ctest --test-dir build -C Release -R "^QIF_Reader\." --output-on-failure
```

The tests carry the same labels as the samples' tests, so `ctest -L exchange` runs them all. The writers' tests read CAD files from
`<HOOPS Exchange>/samples/data`.

## Test data and third-party material

- `QIF_Reader/testdata` holds a synthetic box, a malformed file and six NIST QIF 3.0 models (see `qif-community-LICENSE.md` there).
- `USD_Reader/testdata` holds small `.usda` files written for these tests.
- No customer files are included.

## License

Private repository; license to be decided before it is shared outside Tech Soft 3D (see `LICENSE`).
