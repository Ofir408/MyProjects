################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../AIPlayer.cpp \
../Board.cpp \
../Cell.cpp \
../CellsSwitcher.cpp \
../Client.cpp \
../GameLogic.cpp \
../GameRunner.cpp \
../HumanPlayer.cpp \
../Main.cpp \
../MiniMaxCell.cpp \
../RemotePlayer.cpp \
../TestBoard.cpp 

OBJS += \
./AIPlayer.o \
./Board.o \
./Cell.o \
./CellsSwitcher.o \
./Client.o \
./GameLogic.o \
./GameRunner.o \
./HumanPlayer.o \
./Main.o \
./MiniMaxCell.o \
./RemotePlayer.o \
./TestBoard.o 

CPP_DEPS += \
./AIPlayer.d \
./Board.d \
./Cell.d \
./CellsSwitcher.d \
./Client.d \
./GameLogic.d \
./GameRunner.d \
./HumanPlayer.d \
./Main.d \
./MiniMaxCell.d \
./RemotePlayer.d \
./TestBoard.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


