find_package(Catch2 3 REQUIRED CONFIG)

include(Catch)

add_subdirectory(closures)
add_subdirectory(pgo)
add_subdirectory(core)
add_subdirectory(align_sessions)
