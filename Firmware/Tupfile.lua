-- Utility functions -----------------------------------------------------------

function run_now(command)
    local handle
    handle = io.popen(command)
    local output = handle:read("*a")
    local rc = {handle:close()}
    return rc[1], output
end

-- If we simply invoke python or python3 on a pristine Windows 10, it will try
-- to open the Microsoft Store which will not work and hang tup instead. The
-- command "python --version" does not open the Microsoft Store.
-- On some systems this may return a python2 command if Python3 is not installed.
function find_python3()
    success, python_version = run_now("python --version 2>&1")
    if success and string.match(python_version, "Python 3") then return "python -B" end
    success, python_version = run_now("python3 --version 2>&1")
    if success and string.match(python_version, "Python 3") then return "python3 -B" end
    error("Python 3 not found.")
end

function add_pkg(pkg)
    if pkg.is_included == true then
        return
    end
    pkg.is_included = true
    for _, file in pairs(pkg.code_files or {}) do
        code_files += (pkg.root or '.')..'/'..file
    end
    for _, dir in pairs(pkg.include_dirs or {}) do
        CFLAGS += '-I'..(pkg.root or '.')..'/'..dir
    end
    tup.append_table(CFLAGS, pkg.cflags or {})
    tup.append_table(LDFLAGS, pkg.ldflags or {})
    for _, pkg in pairs(pkg.include or {}) do
        add_pkg(pkg)
    end
end

function compile(src_file, obj_file)
    compiler = (tup.ext(src_file) == 'c') and CC or CXX
    tup.frule{
        inputs={src_file},
        command='^o^ '..compiler..' -c %f '..tostring(CFLAGS)..' -o %o',
        outputs={obj_file}
    }
end

-- Packages --------------------------------------------------------------------

zfoc_firmware_pkg = {
    root = '.',
    include_dirs = {
        '.',
        'Drivers/STM32',
        'Interface',
        'MotorControl',
        'communication',
        'communication/can',
    },
    code_files = {
        'MotorControl/axis.cpp',
        'MotorControl/controller.cpp',
        'MotorControl/encoder.cpp',
        'MotorControl/foc.cpp',
        'MotorControl/low_level.cpp',
        'MotorControl/main.cpp',
        'MotorControl/motor.cpp',
        'MotorControl/open_loop_controller.cpp',
        'MotorControl/trapTraj.cpp',
        'MotorControl/utils.cpp',
        'Drivers/STM32/stm32_system.cpp',
        'Drivers/STM32/stm32_gpio.cpp',
        'Drivers/STM32/stm32_nvm.c',
        'Drivers/STM32/stm32_uart.cpp',
        'Drivers/STM32/cordic_cos_sin.cpp',
        'communication/communication.cpp',
        'communication/can/can_simple.cpp',
        'communication/can/zfoc_can.cpp',
        'Board/version.c',
    }
}

stm32g4xx_hal_pkg = {
    root = 'ThirdParty/STM32G4xx_HAL_Driver',
    include_dirs = {
        'Inc',
    },
    code_files = {
        'Src/stm32g4xx_hal_adc_ex.c',
        'Src/stm32g4xx_hal_adc.c',
        'Src/stm32g4xx_hal_cordic.c',
        'Src/stm32g4xx_hal_cortex.c',
        'Src/stm32g4xx_hal_crc_ex.c',
        'Src/stm32g4xx_hal_crc.c',
        'Src/stm32g4xx_hal_dma_ex.c',
        'Src/stm32g4xx_hal_dma.c',
        'Src/stm32g4xx_hal_exti.c',
        'Src/stm32g4xx_hal_fdcan.c',
        'Src/stm32g4xx_hal_flash_ex.c',
        'Src/stm32g4xx_hal_flash_ramfunc.c',
        'Src/stm32g4xx_hal_flash.c',
        'Src/stm32g4xx_hal_fmac.c',
        'Src/stm32g4xx_hal_gpio.c',
        'Src/stm32g4xx_hal_hrtim.c',
        'Src/stm32g4xx_hal_pwr_ex.c',
        'Src/stm32g4xx_hal_pwr.c',
        'Src/stm32g4xx_hal_rcc_ex.c',
        'Src/stm32g4xx_hal_rcc.c',
        'Src/stm32g4xx_hal_tim_ex.c',
        'Src/stm32g4xx_hal_tim.c',
        'Src/stm32g4xx_hal_uart_ex.c',
        'Src/stm32g4xx_hal_uart.c',
        'Src/stm32g4xx_hal.c',
        'Src/stm32g4xx_ll_adc.c'
    },
    cflags = {
        '-mcpu=cortex-m4',
        '-mfpu=fpv4-sp-d16',
    }
}

freertos_pkg = {
    root = 'ThirdParty/FreeRTOS',
    include_dirs = {
        'Source/include',
        'Source/CMSIS_RTOS',
    },
    code_files = {
        'Source/croutine.c',
        'Source/event_groups.c',
        'Source/list.c',
        'Source/queue.c',
        'Source/stream_buffer.c',
        'Source/tasks.c',
        'Source/timers.c',
        'Source/CMSIS_RTOS/cmsis_os.c',
        'Source/portable/MemMang/heap_4.c',
    }
}

cmsis_pkg = {
    root = 'ThirdParty/CMSIS',
    include_dirs = {
        'Include',
        'Device/ST/STM32G4xx/Include',
    },
    ldflags = {
        '-LThirdParty/CMSIS/Lib/GCC',
    }
}

board_v1 = {
    root = 'Board/v1',
    include = {stm32g4xx_hal_pkg},
    include_dirs = {
        'Inc',
        '../../ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F',
    },
    code_files = {
        'startup_stm32g474xx.s',
        '../../ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c',
        'board.cpp',
        'Src/adc.c',
        'Src/app_freertos.c',
        'Src/cordic.c',
        'Src/crc.c',
        'Src/fdcan.c',
        'Src/fmac.c',
        'Src/gpio.c',
        'Src/hrtim.c',
        'Src/main.c',
        'Src/stm32g4xx_hal_msp.c',
        'Src/stm32g4xx_hal_timebase_tim.c',
        'Src/stm32g4xx_it.c',
        'Src/syscalls.c',
        'Src/sysmem.c',
        'Src/system_stm32g4xx.c',
        'Src/tim.c',
        'Src/usart.c',
        'Src/dma.c',
    },
    cflags = {
        '-DSTM32G474xx',
        '-DHW_VERSION_MAJOR=1',
    },
    ldflags = {
        '-TBoard/v1/Corrected_STM32G474CETx_FLASH.ld',
    }
}

boards = {
    ["v1.1"] = {include={board_v1}, cflags={"-DHW_VERSION_MINOR=1 -DHW_VERSION_VOLTAGE=24"}},
}

-- Toolchain setup -------------------------------------------------------------

CCPATH = tup.getconfig('ARM_COMPILER_PATH')
if CCPATH == "" then
    CCPATH=''
else
    CCPATH = CCPATH..'/'
end

CC=CCPATH..'arm-none-eabi-gcc -std=c99'
CXX=CCPATH..'arm-none-eabi-g++ -std=c++17 -Wno-register'
LINKER=CCPATH..'arm-none-eabi-g++'

-- C-specific flags
CFLAGS += '-D__weak="__attribute__((weak))"'
CFLAGS += '-D__packed="__attribute__((__packed__))"'
CFLAGS += '-DUSE_HAL_DRIVER'

CFLAGS += '-mthumb'
CFLAGS += '-mfloat-abi=hard'
CFLAGS += '-Wno-psabi' -- suppress unimportant note about ABI compatibility in GCC 10
CFLAGS += { '-Wall', '-Wdouble-promotion', '-Wfloat-conversion', '-fdata-sections', '-ffunction-sections'}
CFLAGS += '-g'
CFLAGS += '-DFIBRE_ENABLE_SERVER'
CFLAGS += '-Wno-nonnull'

-- linker flags
LDFLAGS += '-flto -lc -lm -lnosys' -- libs
-- LDFLAGS += '-mthumb -mfloat-abi=hard -specs=nosys.specs -specs=nano.specs -u _printf_float -u _scanf_float -Wl,--cref -Wl,--gc-sections'
LDFLAGS += '-mthumb -mfloat-abi=hard -specs=nosys.specs -u _printf_float -u _scanf_float -Wl,--cref -Wl,--gc-sections'
LDFLAGS += '-Wl,--undefined=uxTopUsedPriority'


-- Handle Configuration Options ------------------------------------------------

-- Switch between board versions
boardversion = tup.getconfig("BOARD_VERSION")
if boardversion == "" then
    error("board version not specified - take a look at tup.config.default")
elseif boards[boardversion] == nil then
    error("unknown board version "..boardversion)
end
board = boards[boardversion]

-- Compiler settings
if tup.getconfig("STRICT") == "true" then
    CFLAGS += '-Werror'
end

if tup.getconfig("NO_DRM") == "true" then
    CFLAGS += '-DNO_DRM'
end

-- debug build
if tup.getconfig("DEBUG") == "true" then
    CFLAGS += '-gdwarf-2 -Og'
else
    CFLAGS += '-O2'
end

if tup.getconfig("USE_LTO") == "true" then
    CFLAGS += '-flto'
end

-- Generate Tup Rules ----------------------------------------------------------

add_pkg(zfoc_firmware_pkg)
add_pkg(freertos_pkg)
add_pkg(cmsis_pkg)
add_pkg(board)

for _, src_file in pairs(code_files) do
    obj_file = "build/obj/"..src_file:gsub("/","_"):gsub("%.","")..".o"
    object_files += obj_file
    compile(src_file, obj_file)
end

tup.frule{
    inputs=object_files,
    command='^o^ '..LINKER..' %f '..tostring(CFLAGS)..' '..tostring(LDFLAGS)..
            ' -Wl,-Map=%O.map -o %o',
    outputs={'build/ZfocFirmware.elf', extra_outputs={'build/ZfocFirmware.map'}}
}
--display the size
tup.frule{
    inputs={'build/ZfocFirmware.elf'},
    command='arm-none-eabi-size %f',
}
