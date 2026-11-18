# Read by ESP-IDF (and so esp-forge) as a component, and by any other CMake
# project that add_subdirectory()s this repo, such as a CubeMX-generated STM32
# one. stm-forge never reads it: it compiles every .c under src/, chip folders
# included, puts src/ on the include path of this library's sources and adds
# include/ -- which is why each port file is empty on the other platform.
# ESP_PLATFORM tells the two apart; IDF sets it in its early requirements pass
# as well as in the build.

if(NOT ESP_PLATFORM)
    # A static library `stepper`. Every .c under src/, as stm-forge does: a
    # chip file compiles to nothing unless its CONFIG_STEPPER_<CHIP> is 1, so
    # there is no source list here to keep in step with the IDF one below.
    file(GLOB_RECURSE stepper_sources CONFIGURE_DEPENDS src/*.c)
    add_library(stepper STATIC ${stepper_sources})
    # include/ for the app; src/ only for this library's own sources, so a
    # chip folder reaches stepper_priv.h by plain name and no private header
    # is ever on the app's include path.
    target_include_directories(stepper PUBLIC include PRIVATE src)
    # lroundf() in the TMC2209 driver.
    target_link_libraries(stepper PUBLIC m)

    # The options are the Kconfig bools, read from Kconfig so a new chip needs
    # no line here: a true CMake variable CONFIG_STEPPER_<CHIP> (or
    # CONFIG_STEPPER_DEBUG) becomes CONFIG_STEPPER_<CHIP>=1, what stm-forge
    # passes. Only bools: an int option cannot be 1-or-absent. Each `config`
    # line in Kconfig is followed directly by its type line, which is what the
    # match relies on. PUBLIC, because stepper_log.h reads
    # CONFIG_STEPPER_DEBUG in the app's sources.
    file(READ Kconfig stepper_kconfig)
    string(REGEX MATCHALL "config STEPPER_[A-Z0-9_]+[ \t\r\n]+bool"
           stepper_options "${stepper_kconfig}")
    list(TRANSFORM stepper_options REPLACE
         "config (STEPPER_[A-Z0-9_]+)[ \t\r\n]+bool" "CONFIG_\\1")
    set(stepper_chips_on "")
    foreach(option IN LISTS stepper_options)
        if(${option})
            target_compile_definitions(stepper PUBLIC ${option}=1)
            if(NOT option STREQUAL "CONFIG_STEPPER_DEBUG")
                list(APPEND stepper_chips_on ${option})
            endif()
        endif()
    endforeach()
    if(NOT stepper_chips_on)
        message(WARNING "stepper: no chip driver is selected; "
                        "set(CONFIG_STEPPER_<CHIP> ON) before add_subdirectory()")
    endif()

    # The HAL is the app's. MCU_HAL_TARGET names a target carrying its include
    # dirs and defines (USE_HAL_DRIVER, the device, the hal_conf.h directory)
    # -- stm32cubemx in a CubeMX-generated project; MCU_DEVICE_HEADER is the
    # family header the public headers include as CMSIS_device_header, which
    # CubeMX does not define. Both are shared by every mcu-* library, so an
    # app sets them once. Left unset, the app provides them its own way.
    #
    # Only the target's include dirs and defines are copied, the target is
    # never linked: CubeMX up to 6.13 also hangs main.c, the HAL and the
    # startup file on stm32cubemx as INTERFACE sources, which linking would
    # compile into libstepper.a. A misspelt name fails at generate time.
    if(MCU_HAL_TARGET)
        target_include_directories(stepper PRIVATE
            $<TARGET_PROPERTY:${MCU_HAL_TARGET},INTERFACE_INCLUDE_DIRECTORIES>)
        target_compile_definitions(stepper PRIVATE
            $<TARGET_PROPERTY:${MCU_HAL_TARGET},INTERFACE_COMPILE_DEFINITIONS>)
    endif()
    if(MCU_DEVICE_HEADER)
        target_compile_definitions(stepper PUBLIC
            "CMSIS_device_header=\"${MCU_DEVICE_HEADER}\"")
    endif()
    return()
endif()

# ESP-IDF: the chip files are listed one by one, and PRIV_INCLUDE_DIRS "src"
# lets a chip folder reach stepper_priv.h by plain name.
#
# The core is always built, even with CONFIG_STEPPER off; each chip driver
# only when its Kconfig option is on. Kconfig values are available here during
# the build phase, and only SRCS depends on them -- never REQUIRES, which IDF
# resolves before sdkconfig. That is also why STEPPER off still builds the
# core: an empty SRCS would need a second, differently-shaped register call,
# and an unreferenced core costs nothing once --gc-sections has run.
#
# REQUIRES driver, not esp_driver_uart/spi/gpio: the split components do not
# exist on IDF 5.2, and driver still pulls them in on later versions.
set(srcs "src/stepper.c" "src/stepper_log.c" "src/stepper_port_esp.c")

if(CONFIG_STEPPER AND NOT (CONFIG_STEPPER_TMC2209 OR CONFIG_STEPPER_TMC5240))
    message(WARNING "stepper: CONFIG_STEPPER is on but no chip driver is "
                    "selected; set CONFIG_STEPPER_<CHIP>=y "
                    "(esp-forge: STEPPER_<CHIP> = y)")
endif()

if(CONFIG_STEPPER_TMC2209)
    list(APPEND srcs "src/tmc2209/tmc2209.c")
endif()
if(CONFIG_STEPPER_TMC5240)
    list(APPEND srcs "src/tmc5240/tmc5240.c")
endif()

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "src"
                       REQUIRES driver)
