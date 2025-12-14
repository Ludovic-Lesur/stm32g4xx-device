# Description

This repository contains the **startup code and linker scripts** of the STM32G4xx MCUs.

# Dependencies

The drivers rely on:

* An external `types.h` header file defining the **standard C types** of the targeted MCU.
* The `main` application function which is called at the end of the **reset handler**.

# Compilation flags

| **Flag name** | **Value** | **Description** |
|:---:|:---:|:---:|
| `STM32G4XX_DEVICE_DISABLE_FLAGS_FILE` | `defined` / `undefined` | Disable the `stm32g4xx_device_flags.h` header file inclusion when compilation flags are given in the project settings or by command line. |
| `STM32G4XX_DEVICE_DISABLE` | `defined` / `undefined` | Disable the STM32G4xx device code. |
| `STM32G4XX_DEVICE_STACK_SIZE` | `<value>` | Size of the stack to reserve in SRAM2 memory. |
| `STM32G4XX_DEVICE_HEAP_SIZE` | `<value>` | Size of the heap to reserve in SRAM2 memory. |

# Build

A static library can be compiled by command line with `cmake`.

```bash
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE="<toolchain_file_path>" \
      -DTOOLCHAIN_PATH="<arm-none-eabi-gcc_path>" \
      -DTYPES_PATH="<types_file_path>" \
      -DSTM32G4XX_DEVICE_STACK_SIZE=0x00000400 \
      -DSTM32G4XX_DEVICE_HEAP_SIZE=0x00000C00 \
      -G "Unix Makefiles" ..
make all
```
