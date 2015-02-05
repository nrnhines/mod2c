include(TestScriptUtils)

set(TEST_LABEL validation)

add_test_class(validation)

set(TEST_ENVIRONMENT "MODLUNIT=${CMAKE_SOURCE_DIR}/share/nrnunits.lib")

foreach(mod2c "mod2c" "mod2c_core")
    set(test_data "${CMAKE_CURRENT_SOURCE_DIR}/${mod2c}")
    file(GLOB test_mods RELATIVE "${test_data}/mod" "${test_data}/mod/*.mod")

    foreach(modfile ${test_mods})
	set(mod ${modfile})
	string(REGEX REPLACE "\\.mod$" "" mod "${modfile}")

	set(TEST_ARGS ${CMAKE_BINARY_DIR}/bin/${mod2c} ${test_data}/mod/${mod}.mod ${test_data}/c/${mod}.c)
	add_test_script("${mod2c}-${mod}" test_mod2c "/bin/bash")
    endforeach()
endforeach()

