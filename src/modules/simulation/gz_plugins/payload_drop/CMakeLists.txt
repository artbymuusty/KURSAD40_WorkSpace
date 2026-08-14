add_library(PayloadDropSystem SHARED payload_drop_system.cc)

target_link_libraries(PayloadDropSystem
    ${GZ_SIM_TARGET}
    ${GZ_TRANSPORT_TARGET}
    ${GZ_PLUGIN_TARGET}
)
