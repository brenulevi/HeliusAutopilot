
# Consider dependencies only in project.
set(CMAKE_DEPENDS_IN_PROJECT_ONLY OFF)

# The set of languages for which implicit dependencies are needed:
set(CMAKE_DEPENDS_LANGUAGES
  "ASM"
  )
# The set of files for implicit dependencies of each language:
set(CMAKE_DEPENDS_CHECK_ASM
  "/home/breno/dev/HeliusAutopilot/Firmware/startup_stm32f411xe.s" "/home/breno/dev/HeliusAutopilot/build/Debug/Firmware/CMakeFiles/Firmware.dir/startup_stm32f411xe.s.obj"
  )
set(CMAKE_ASM_COMPILER_ID "GNU")

# Preprocessor definitions for this target.
set(CMAKE_TARGET_DEFINITIONS_ASM
  "DEBUG"
  "STM32F411xE"
  "USE_HAL_DRIVER"
  )

# The include file search paths:
set(CMAKE_ASM_TARGET_INCLUDE_PATH
  "../../Firmware/Drivers/MPU9250"
  "../../Firmware/Drivers/Protocol"
  "../../Firmware/cmake/stm32cubemx/../../USB_DEVICE/App"
  "../../Firmware/cmake/stm32cubemx/../../USB_DEVICE/Target"
  "../../Firmware/cmake/stm32cubemx/../../Core/Inc"
  "../../Firmware/cmake/stm32cubemx/../../Drivers/STM32F4xx_HAL_Driver/Inc"
  "../../Firmware/cmake/stm32cubemx/../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy"
  "../../Firmware/cmake/stm32cubemx/../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc"
  "../../Firmware/cmake/stm32cubemx/../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc"
  "../../Firmware/cmake/stm32cubemx/../../Drivers/CMSIS/Device/ST/STM32F4xx/Include"
  "../../Firmware/cmake/stm32cubemx/../../Drivers/CMSIS/Include"
  "../../Core/include"
  )

# The set of dependency files which are needed:
set(CMAKE_DEPENDS_DEPENDENCY_FILES
  "/home/breno/dev/HeliusAutopilot/Firmware/Core/Src/main.c" "Firmware/CMakeFiles/Firmware.dir/Core/Src/main.c.obj" "gcc" "Firmware/CMakeFiles/Firmware.dir/Core/Src/main.c.obj.d"
  "/home/breno/dev/HeliusAutopilot/Firmware/Core/Src/stm32f4xx_hal_msp.c" "Firmware/CMakeFiles/Firmware.dir/Core/Src/stm32f4xx_hal_msp.c.obj" "gcc" "Firmware/CMakeFiles/Firmware.dir/Core/Src/stm32f4xx_hal_msp.c.obj.d"
  "/home/breno/dev/HeliusAutopilot/Firmware/Core/Src/stm32f4xx_it.c" "Firmware/CMakeFiles/Firmware.dir/Core/Src/stm32f4xx_it.c.obj" "gcc" "Firmware/CMakeFiles/Firmware.dir/Core/Src/stm32f4xx_it.c.obj.d"
  "/home/breno/dev/HeliusAutopilot/Firmware/Core/Src/syscalls.c" "Firmware/CMakeFiles/Firmware.dir/Core/Src/syscalls.c.obj" "gcc" "Firmware/CMakeFiles/Firmware.dir/Core/Src/syscalls.c.obj.d"
  "/home/breno/dev/HeliusAutopilot/Firmware/Core/Src/sysmem.c" "Firmware/CMakeFiles/Firmware.dir/Core/Src/sysmem.c.obj" "gcc" "Firmware/CMakeFiles/Firmware.dir/Core/Src/sysmem.c.obj.d"
  "/home/breno/dev/HeliusAutopilot/Firmware/Drivers/MPU9250/mpu9250.c" "Firmware/CMakeFiles/Firmware.dir/Drivers/MPU9250/mpu9250.c.obj" "gcc" "Firmware/CMakeFiles/Firmware.dir/Drivers/MPU9250/mpu9250.c.obj.d"
  "/home/breno/dev/HeliusAutopilot/Firmware/Drivers/Protocol/i2c_protocol.c" "Firmware/CMakeFiles/Firmware.dir/Drivers/Protocol/i2c_protocol.c.obj" "gcc" "Firmware/CMakeFiles/Firmware.dir/Drivers/Protocol/i2c_protocol.c.obj.d"
  "/home/breno/dev/HeliusAutopilot/Firmware/USB_DEVICE/App/usb_device.c" "Firmware/CMakeFiles/Firmware.dir/USB_DEVICE/App/usb_device.c.obj" "gcc" "Firmware/CMakeFiles/Firmware.dir/USB_DEVICE/App/usb_device.c.obj.d"
  "/home/breno/dev/HeliusAutopilot/Firmware/USB_DEVICE/App/usbd_cdc_if.c" "Firmware/CMakeFiles/Firmware.dir/USB_DEVICE/App/usbd_cdc_if.c.obj" "gcc" "Firmware/CMakeFiles/Firmware.dir/USB_DEVICE/App/usbd_cdc_if.c.obj.d"
  "/home/breno/dev/HeliusAutopilot/Firmware/USB_DEVICE/App/usbd_desc.c" "Firmware/CMakeFiles/Firmware.dir/USB_DEVICE/App/usbd_desc.c.obj" "gcc" "Firmware/CMakeFiles/Firmware.dir/USB_DEVICE/App/usbd_desc.c.obj.d"
  "/home/breno/dev/HeliusAutopilot/Firmware/USB_DEVICE/Target/usbd_conf.c" "Firmware/CMakeFiles/Firmware.dir/USB_DEVICE/Target/usbd_conf.c.obj" "gcc" "Firmware/CMakeFiles/Firmware.dir/USB_DEVICE/Target/usbd_conf.c.obj.d"
  )

# Targets to which this target links.
set(CMAKE_TARGET_LINKED_INFO_FILES
  "/home/breno/dev/HeliusAutopilot/build/Debug/Core/CMakeFiles/helius_core.dir/DependInfo.cmake"
  )

# Fortran module output directory.
set(CMAKE_Fortran_TARGET_MODULE_DIR "")
