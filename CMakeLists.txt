zephyr_include_directories(include)

if(CONFIG_ZMK_PSP_JOYSTICK)
    target_sources(app PRIVATE
        src/psp_joystick_math.c
        src/psp_joystick_state.c
    )

    if(CONFIG_ZMK_PSP_JOYSTICK_DRIVER)
        target_sources(app PRIVATE drivers/input/input_psp_joystick.c)
    endif()

    if(CONFIG_ZMK_PSP_JOYSTICK_SPLIT_RELAY)
        target_sources(app PRIVATE src/split/psp_joystick_relay.c)
    endif()

    if(CONFIG_ZMK_PSP_JOYSTICK_STUDIO_RPC)
        target_sources(app PRIVATE
            src/studio/psp_joystick_handler.c
            src/studio/psp_joystick_settings.c
        )

        list(APPEND CMAKE_MODULE_PATH ${ZEPHYR_BASE}/modules/nanopb)
        include(nanopb)
        set(NANOPB_GENERATE_CPP_APPEND_PATH TRUE)
        set(NANOPB_GENERATE_CPP_STANDALONE OFF)

        zephyr_library()
        file(GLOB_RECURSE PROTO_FILES ${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto)
        nanopb_generate_cpp(proto_srcs proto_hdrs RELPATH ${CMAKE_CURRENT_SOURCE_DIR} ${PROTO_FILES})
        target_include_directories(${ZEPHYR_CURRENT_LIBRARY} PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
        target_sources(${ZEPHYR_CURRENT_LIBRARY} PRIVATE ${proto_srcs} ${proto_hdrs})

        target_include_directories(app PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/proto)
        target_link_libraries(app PRIVATE ${ZEPHYR_CURRENT_LIBRARY})
    endif()
endif()
