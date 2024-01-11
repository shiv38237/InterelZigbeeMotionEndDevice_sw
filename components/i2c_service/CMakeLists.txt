set(srcs "src/i2c_service.c")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS include
REQUIRES driver i2c_service)
