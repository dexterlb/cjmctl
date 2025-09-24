cmake_minimum_required(VERSION 3.11)

project(tester)

set(SOURCES
    main.c
)

add_executable(${CMAKE_PROJECT_NAME} ${SOURCES})
set_property(TARGET ${CMAKE_PROJECT_NAME} PROPERTY C_STANDARD 23)
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Werror
    -Wall -W -Wextra -Wpedantic
    -Wbad-function-cast -Wcast-align -Wcast-qual -Wchar-subscripts
    -Winline -Wnested-externs -Wpointer-arith
    -Wredundant-decls -Wshadow -Wstrict-prototypes -Wwrite-strings
)

add_subdirectory(../../ cjmctl_build)

target_link_libraries(${CMAKE_PROJECT_NAME}
    cjmctl
)
