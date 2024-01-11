set(srcs "src/pyq1548.c")

idf_component_register(SRCS "${srcs}" 
                    INCLUDE_DIRS include
                    REQUIRES driver
)