find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
 
include_directories(${PROJECT_SOURCE_DIR}/glc/include)

add_executable(test_a test.cpp)
target_link_libraries(test_a glc_node ${GTEST_BOTH_LIBRARIES})
add_test(gtest_a test_a)

#Unit Test
add_executable(test_glc_math test_glc_math.cpp)
target_link_libraries(test_glc_math
                      ${GTEST_BOTH_LIBRARIES} 
                      glc_math)
add_test(gtest_glc_math test_glc_math)

#System Test
add_executable(base-test base_test.cpp)
target_link_libraries(base-test 
                      ${GTEST_BOTH_LIBRARIES} 
                      glc_planner_core)                      
add_test(gtest-base-test base-test)

install(TARGETS test_a
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)