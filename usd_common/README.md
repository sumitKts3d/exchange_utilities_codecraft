# usd_common — shared by USD_Reader (USD → PRC) and USD_Writer (CAD/PRC → USD)

| File | What it holds |
| --- | --- |
| `OpenUSD.cmake` | the OpenUSD folder (`<HOOPS Exchange>/bin/usd`, else `USD_ROOT`; resolved by `OpenUSDRoot.cmake`), the imported OpenUSD targets (with TBB and Python described), `hoops_usd_setup(<target> <libs>)` (links, include path, MSVC options, copies the OpenUSD runtime and `usd/` plug-ins next to the executable), `HOOPS_USD_TOOLS_ENV` for tests that run `usdcat` / `usdchecker` |
| `OpenUSDRoot.cmake` | `hoops_usd_resolve_root(<var>)`: looks for OpenUSD in `<HOOPS Exchange>/bin/usd`, then in `USD_ROOT` |
| `hoops_usd.hpp` | conventions both directions must agree on: GfMatrix4d ↔ PRC transformation, units, Y-up → Z-up, `Look` and roughness ↔ shininess, prim names and attribute namespaces (`userProperties:`, `hoops:`), HOOPS Exchange session and messages, exit codes; PRC helpers: decoding of every tessellation layout, the SDK's product-structure rules (effective part / children / location), tree dump, bounding box |

Use from an app's `CMakeLists.txt` (the top-level `CMakeLists.txt` resolves the OpenUSD folder first):

```cmake
include("${CMAKE_CURRENT_SOURCE_DIR}/../usd_common/OpenUSD.cmake")
target_link_libraries(MyApp PRIVATE A3DLIBS::Interface)
hoops_usd_setup(MyApp usd usdGeom usdShade sdf tf vt gf)
```

and in the one source file that defines `INITIALIZE_A3D_API`, include `hoops_usd.hpp` after `<A3DSDKIncludes.h>`.
