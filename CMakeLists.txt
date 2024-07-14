cmake_minimum_required(VERSION 3.15)

project(DirectionalSpeaker LANGUAGES C CXX ASM)

find_bsp(
	ID com.sysprogs.project_importers.stm32.cubemx
	SOURCE_PROJECT DirectionalSpeaker.ioc
	HWREGISTER_LIST_FILE C:/Users/PRUEK/AppData/Local/VisualGDB/EmbeddedBSPs/arm-eabi/com.sysprogs.arm.stm32/STM32F1xxxx/DeviceDefinitions/stm32f103x6.xml.gz)

add_bsp_based_executable(
	NAME DirectionalSpeaker
	GENERATE_BIN
	GENERATE_MAP)

