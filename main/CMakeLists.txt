idf_component_register(SRCS "usb_descriptors.c" "main.c" "board.c"
                      INCLUDE_DIRS "."
                      PRIV_REQUIRES driver usb
                      REQUIRES tinyusb)


# Workaround linker order for TinyUSB descriptor callbacks.
# Force the final link to include these symbols from this component
# so espressif__tinyusb can resolve them when it’s linked later.
target_link_options(${COMPONENT_LIB} INTERFACE
    "-Wl,-u,tud_descriptor_device_cb"
    "-Wl,-u,tud_descriptor_configuration_cb"
    "-Wl,-u,tud_descriptor_string_cb")


