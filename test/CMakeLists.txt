set(proj_source_path ../src/${CamelCaseName}/)
include_directories(${proj_source_path} ${CMAKE_CURRENT_BINARY_DIR}/${proj_source_path})

set(REGR_TESTS
    dataset_UNIT.cpp
    offset_estimator_UNIT.cpp
    force_calibration_matrix_estimator_UNIT.cpp
    #calibration_matrix_estimator_UNIT.cpp
    )

foreach(TEST_PROG ${REGR_TESTS})

    get_filename_component(TEST_ROOT ${TEST_PROG} NAME_WE)

    add_executable(${TEST_ROOT}_test ${TEST_PROG})
    target_link_libraries(${TEST_ROOT}_test ${library_target_name})
    add_test(NAME test_${TEST_ROOT} COMMAND ${TEST_ROOT}_test)

endforeach()