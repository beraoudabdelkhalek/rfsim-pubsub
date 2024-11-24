# Find the ZeroMQ package
find_package(ZeroMQ REQUIRED)

# Add the rfsimulator library
add_library(rfsimulator MODULE
            simulator.c
            apply_channelmod.c
            ../../openair1/PHY/TOOLS/signal_energy.c
)

# Link ZeroMQ to the rfsimulator library
target_link_libraries(rfsimulator PRIVATE SIMU ZeroMQ::libzmq)

# Set the output directory for the library
set_target_properties(rfsimulator PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# Add the replay_node executable
add_executable(replay_node stored_node.c)

# Link minimal_lib to the replay_node executable
target_link_libraries(replay_node PRIVATE minimal_lib)