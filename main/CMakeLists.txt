set(srcs "main.c"
        "board.c")

idf_component_register(SRCS "display_adc.c" "loop.c" "${srcs}"
                    INCLUDE_DIRS  ".")
