set(NOCMODL_SOURCES
"consist.c"
"deriv.c"
"discrete.c"
"init.c"
"io.c"
"kinetic.c"
"list.c"
"modl.c"
"noccout.c"
"nocpout.c"
"parsact.c"
"partial.c"
"sens.c"
"simultan.c"
"solve.c"
"symbol.c"
"units.c"
"version.c"
)

bison_target(diffeq diffeq.y ${CMAKE_CURRENT_BINARY_DIR}/diffeq.c)
bison_target(parse1 parse1.y ${CMAKE_CURRENT_BINARY_DIR}/parse1.c)

flex_target(lex lex.l ${CMAKE_CURRENT_BINARY_DIR}/lex.c)
add_flex_bison_dependency(lex parse1)

add_definitions(-DNMODL=1 -DNOCMODL=1 -DCVODE=1 -DVECTORIZE=1)
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

add_executable(mod2c
    ${NOCMODL_SOURCES}
    ${BISON_diffeq_OUTPUTS} ${BISON_parse1_OUTPUTS}
    ${FLEX_lex_OUTPUTS})

install(TARGETS mod2c DESTINATION bin)

