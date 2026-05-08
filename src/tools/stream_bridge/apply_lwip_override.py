from pathlib import Path
import shutil

from SCons.Script import DefaultEnvironment


env = DefaultEnvironment()

platform = env.PioPlatform()
framework_dir = Path(platform.get_package_dir("framework-arduinoespressif8266"))
if not framework_dir:
    raise RuntimeError("framework-arduinoespressif8266 package not found")

repo_root = Path(env["PROJECT_DIR"])
override_dir = repo_root / "tools" / "stream_bridge" / "lwip"
sdk_lib_dir = framework_dir / "tools" / "sdk" / "lib"
lwip_builder_dir = framework_dir / "tools" / "sdk" / "lwip2" / "builder"
lwip_include_dir = framework_dir / "tools" / "sdk" / "lwip2" / "include"

for name in ("liblwip2-1460-feat.a", "liblwip2-1460.a"):
    src = override_dir / name
    dst = sdk_lib_dir / name
    if not src.exists():
        raise RuntimeError(f"Missing lwIP override library: {src}")
    shutil.copy2(src, dst)
    print(f"[stream_bridge] override {name}")

cmake_src = override_dir / "CMakeLists.txt"
cmake_dst = lwip_builder_dir / "CMakeLists.txt"
if not cmake_src.exists():
    raise RuntimeError(f"Missing lwIP override CMakeLists.txt: {cmake_src}")
shutil.copy2(cmake_src, cmake_dst)
print("[stream_bridge] override lwip2 builder CMakeLists.txt")

lwipopts_src = override_dir / "lwipopts.h"
lwipopts_dst = lwip_include_dir / "lwipopts.h"
if not lwipopts_src.exists():
    raise RuntimeError(f"Missing lwIP override lwipopts.h: {lwipopts_src}")
shutil.copy2(lwipopts_src, lwipopts_dst)
print("[stream_bridge] override lwipopts.h")
