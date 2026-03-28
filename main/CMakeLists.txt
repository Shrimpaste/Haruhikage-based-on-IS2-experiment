idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS ".")

# 链接数学库
target_link_libraries(${COMPONENT_LIB} m)
