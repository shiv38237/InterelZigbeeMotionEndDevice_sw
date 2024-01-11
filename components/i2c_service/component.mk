#
# Component Makefile
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)
#COMPONENT_ADD_INCLUDEDIRS is enabled by default to remove stub implementation dependencies and COMPONENT_SRCDIRS
#will be compiled based on menucofing selection otherwise its stub will be compiled.
COMPONENT_ADD_INCLUDEDIRS := include
ifdef CONFIG_I2C_SERVICE
COMPONENT_SRCDIRS := src
endif