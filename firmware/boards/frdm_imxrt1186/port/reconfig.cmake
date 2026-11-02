# Board port for BUSBRIDGE-RT. The per-core directory holds hardware_init.c,
# which is where the SDK's generated clock and pin setup is called from.
include(${CMAKE_CURRENT_LIST_DIR}/${core_id}/reconfig.cmake OPTIONAL)
