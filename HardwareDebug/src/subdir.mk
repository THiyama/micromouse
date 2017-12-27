################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
..\src/aaa.c \
..\src/adjust.c \
..\src/contest.c \
..\src/dbsct.c \
..\src/extern_declaration.c \
..\src/hardware_setup.c \
..\src/interrupt_handlers.c \
..\src/motor.c \
..\src/reset_program.c \
..\src/rx631.c \
..\src/rx631_init.c \
..\src/sbrk.c \
..\src/search.c \
..\src/serial.c \
..\src/shortest_run.c \
..\src/vector_table.c \
..\src/wall_control.c 

C_DEPS += \
./src/aaa.d \
./src/adjust.d \
./src/contest.d \
./src/dbsct.d \
./src/extern_declaration.d \
./src/hardware_setup.d \
./src/interrupt_handlers.d \
./src/motor.d \
./src/reset_program.d \
./src/rx631.d \
./src/rx631_init.d \
./src/sbrk.d \
./src/search.d \
./src/serial.d \
./src/shortest_run.d \
./src/vector_table.d \
./src/wall_control.d 

OBJS += \
./src/aaa.obj \
./src/adjust.obj \
./src/contest.obj \
./src/dbsct.obj \
./src/extern_declaration.obj \
./src/hardware_setup.obj \
./src/interrupt_handlers.obj \
./src/motor.obj \
./src/reset_program.obj \
./src/rx631.obj \
./src/rx631_init.obj \
./src/sbrk.obj \
./src/search.obj \
./src/serial.obj \
./src/shortest_run.obj \
./src/vector_table.obj \
./src/wall_control.obj 


# Each subdirectory must supply rules for building sources it contributes
src/%.obj: ../src/%.c
	@echo 'Scanning and building file: $<'
	@echo 'Invoking: Scanner and Compiler'
	ccrx  -MM -MP -output=dep="$(@:%.obj=%.d)" -MT="$(@:%.obj=%.obj)" -MT="$(@:%.obj=%.d)" -lang=c   -include="C:\PROGRA~2\Renesas\RX\2_4_1/include"  -debug -nomessage -isa=rxv1 -optimize=0 -fpu -alias=noansi -nologo  -define=__RX   "$<"
	ccrx -lang=c -output=obj="$(@:%.d=%.obj)"  -include="C:\PROGRA~2\Renesas\RX\2_4_1/include"  -debug -nomessage -isa=rxv1 -optimize=0 -fpu -alias=noansi -nologo  -define=__RX   "$<"
	@echo 'Finished scanning and building: $<'
	@echo.

