#!/usr/bin/env python3
"""Build the CubeIDE firmware with the installed ARM GCC, without launching IDE."""
import argparse
import re
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gcc", type=Path, help="Path to arm-none-eabi-gcc")
    parser.add_argument("--output", type=Path, default=ROOT / "build-compass")
    args = parser.parse_args()
    gcc = args.gcc or shutil.which("arm-none-eabi-gcc")
    if gcc is None:
        candidates = sorted(Path("/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins").glob(
            "com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin/arm-none-eabi-gcc"))
        gcc = candidates[-1] if candidates else None
    if gcc is None:
        parser.error("ARM GCC not found; pass --gcc /path/to/arm-none-eabi-gcc")
    gcc = Path(gcc).resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    target = ["-mcpu=cortex-m4", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard", "-mthumb"]
    includes = ["Core/Inc", "Drivers/STM32F4xx_HAL_Driver/Inc",
                "Drivers/STM32F4xx_HAL_Driver/Inc/Legacy", "Drivers/CMSIS/Include",
                "Drivers/CMSIS/Device/ST/STM32F4xx/Include"]
    common = target + ["-g3", "-O0", "-ffunction-sections", "-fdata-sections",
                       "-DUSE_HAL_DRIVER", "-DSTM32F427xx", "-DDEBUG", "--specs=nano.specs"]
    common += ["-I" + str(ROOT / path) for path in includes]
    enabled = set(re.findall(r"^#define HAL_(\w+)_MODULE_ENABLED", (
        ROOT / "Core/Inc/stm32f4xx_hal_conf.h").read_text(), re.MULTILINE))
    sources = sorted(path for path in (ROOT / "Core/Src").glob("*.c") if path.name != "lab3.c")
    for path in sorted((ROOT / "Drivers/STM32F4xx_HAL_Driver/Src").glob("*.c")):
        name = path.stem
        module = name.removeprefix("stm32f4xx_hal_").split("_")[0].upper()
        if name == "stm32f4xx_hal" or module in enabled:
            sources.append(path)
    sources += sorted((ROOT / "Core/Startup").glob("*.s"))
    objects = []
    for source in sources:
        obj = output / source.relative_to(ROOT).with_suffix(".o")
        obj.parent.mkdir(parents=True, exist_ok=True)
        flags = ["-std=gnu11", "-Wall"] if source.suffix == ".c" else []
        # Treat warnings in application code as errors, without changing ST sources.
        if source.parent == ROOT / "Core/Src" and source.name in {
                "compass.c", "compass_math.c", "compass_calibration.c",
                "magnetometer.c", "oled.c", "wire_protocol.c"}:
            flags += ["-Wextra", "-Werror"]
        subprocess.run([str(gcc), *common, *flags, "-c", str(source), "-o", str(obj)], check=True)
        objects.append(str(obj))
    elf = output / "compass.elf"
    subprocess.run([str(gcc), *target, "--specs=nano.specs", "--specs=nosys.specs",
                    "-T" + str(ROOT / "STM32F427VITX_FLASH.ld"), "-Wl,--gc-sections",
                    "-Wl,-Map=" + str(output / "compass.map"), "-o", str(elf), *objects,
                    "-Wl,--start-group", "-lc", "-lm", "-Wl,--end-group"], check=True)
    suffix = ".exe" if gcc.suffix == ".exe" else ""
    subprocess.run([str(gcc.with_name("arm-none-eabi-size" + suffix)), str(elf)], check=True)
    subprocess.run([str(gcc.with_name("arm-none-eabi-objcopy" + suffix)), "-O", "ihex",
                    str(elf), str(output / "compass.hex")], check=True)
    print(f"Built {elf}")


if __name__ == "__main__":
    main()
