# This file is for ESP-IDF only
idf_component_register(SRCS "main.c" "usb_descriptors.c" "audio_hw.c" "pcm1864.c"
                    INCLUDE_DIRS "."
                    REQUIRES boards driver tinyusb_src)
