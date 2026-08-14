add_library(HookAttachSystem SHARED HookAttachSystem.cc)

target_link_libraries(HookAttachSystem
    ${GZ_SIM_TARGET}
    ${GZ_TRANSPORT_TARGET}
    ${GZ_PLUGIN_TARGET}
)
