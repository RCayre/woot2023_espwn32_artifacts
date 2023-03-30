idf_component_register(SRCS "esperanto.c" "hci.c" "helpers.c"
                       INCLUDE_DIRS "include" "$ENV{IDF_PATH}/components/log/include/" "$ENV{IDF_PATH}/components/bt/include/${IDF_TARGET}/include/"
                       PRIV_REQUIRES nvs_flash
)
string(TOUPPER ${IDF_TARGET} TARGET)
target_compile_options(${COMPONENT_LIB} PRIVATE -D${TARGET})
