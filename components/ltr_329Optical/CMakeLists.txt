set(srcs "src/ltr_329Optical.c")

idf_component_register(SRCS "${srcs}" 
                    INCLUDE_DIRS include
                    REQUIRES driver i2c_service
)