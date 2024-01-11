set(srcs "src/STHS34PF80.c")

idf_component_register(SRCS "${srcs}" 
                    INCLUDE_DIRS include
                    REQUIRES driver i2c_service pyq1548
)