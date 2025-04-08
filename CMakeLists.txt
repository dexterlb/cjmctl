add_library(cjmctl)

target_sources(cjmctl PRIVATE
    src/control_position.c
    src/control_velocity.c
    src/estimator_velocity_dumb.c
    src/math_utils.c
)

target_include_directories(cjmctl INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

set_property(TARGET cjmctl PROPERTY C_STANDARD 23)
