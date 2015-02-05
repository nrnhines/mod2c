set(NOCMODL_CORE_SOURCES
"consist.c"
"io.c"       "list.c"
"nocpout.c"  "partial.c"
"solve.c"    "version.c"
"discrete.c"
"kinetic.c"  "modl.c"
"parsact.c"  "sens.c"
"symbol.c"   "deriv.c"
"init.c"
"noccout.c"
"simultan.c" "units.c"
)

bison_target(diffeq diffeq.y ${CMAKE_CURRENT_BINARY_DIR}/diffeq.c)
bison_target(parse1 parse1.y ${CMAKE_CURRENT_BINARY_DIR}/parse1.c)

flex_target(lex lex.l ${CMAKE_CURRENT_BINARY_DIR}/lex.c)
add_flex_bison_dependency(lex parse1)

add_definitions(-DNMODL=1 -DBBCORE=1 -DNOCMODL=1 -DCVODE=1 -DVECTORIZE=1)
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

add_executable(mod2c_core
    ${NOCMODL_CORE_SOURCES}
    ${BISON_diffeq_OUTPUTS} ${BISON_parse1_OUTPUTS}
    ${FLEX_lex_OUTPUTS})

install(TARGETS mod2c_core DESTINATION bin)

