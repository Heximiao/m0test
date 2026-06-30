################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
SYSCFG_SRCS += \
../gpio_toggle_output.syscfg 

C_SRCS += \
../app_car_control.c \
../bsp_tb6612.c \
../gpio_toggle_output.c \
./ti_msp_dl_config.c \
C:/TI/mspm0_sdk_2_10_00_04/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c \
../hw_encoder.c \
../hw_uart.c \
../mid_pid.c 

GEN_CMDS += \
./device_linker.cmd 

GEN_FILES += \
./device_linker.cmd \
./device.opt \
./ti_msp_dl_config.c 

C_DEPS += \
./app_car_control.d \
./bsp_tb6612.d \
./gpio_toggle_output.d \
./ti_msp_dl_config.d \
./startup_mspm0g350x_ticlang.d \
./hw_encoder.d \
./hw_uart.d \
./mid_pid.d 

GEN_OPTS += \
./device.opt 

OBJS += \
./app_car_control.o \
./bsp_tb6612.o \
./gpio_toggle_output.o \
./ti_msp_dl_config.o \
./startup_mspm0g350x_ticlang.o \
./hw_encoder.o \
./hw_uart.o \
./mid_pid.o 

GEN_MISC_FILES += \
./device.cmd.genlibs \
./ti_msp_dl_config.h \
./Event.dot 

OBJS__QUOTED += \
"app_car_control.o" \
"bsp_tb6612.o" \
"gpio_toggle_output.o" \
"ti_msp_dl_config.o" \
"startup_mspm0g350x_ticlang.o" \
"hw_encoder.o" \
"hw_uart.o" \
"mid_pid.o" 

GEN_MISC_FILES__QUOTED += \
"device.cmd.genlibs" \
"ti_msp_dl_config.h" \
"Event.dot" 

C_DEPS__QUOTED += \
"app_car_control.d" \
"bsp_tb6612.d" \
"gpio_toggle_output.d" \
"ti_msp_dl_config.d" \
"startup_mspm0g350x_ticlang.d" \
"hw_encoder.d" \
"hw_uart.d" \
"mid_pid.d" 

GEN_FILES__QUOTED += \
"device_linker.cmd" \
"device.opt" \
"ti_msp_dl_config.c" 

C_SRCS__QUOTED += \
"../app_car_control.c" \
"../bsp_tb6612.c" \
"../gpio_toggle_output.c" \
"./ti_msp_dl_config.c" \
"C:/TI/mspm0_sdk_2_10_00_04/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c" \
"../hw_encoder.c" \
"../hw_uart.c" \
"../mid_pid.c" 

SYSCFG_SRCS__QUOTED += \
"../gpio_toggle_output.syscfg" 


