find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
 
include_directories(${PROJECT_SOURCE_DIR}/glc/include)

add_executable(test_glc_node test_glc_node.cpp)
target_link_libraries(test_glc_node 
                      glc_node 
                      ${GTEST_BOTH_LIBRARIES})
add_test(gtest_glc_node test_glc_node)

add_executable(test_glc_math test_glc_math.cpp)
target_link_libraries(test_glc_math
                      ${GTEST_BOTH_LIBRARIES} 
                      glc_math)
add_test(gtest_glc_math test_glc_math)

add_executable(test_glc_equivalence_relation test_glc_equivalence_relation.cpp)
target_link_libraries(test_glc_equivalence_relation 
                      glc_state_equivalence_class
                      ${GTEST_BOTH_LIBRARIES})
add_test(gtest_glc_equivalence_relation test_glc_equivalence_relation)

#System Test
add_executable(test_glc_planner_core test_glc_planner_core.cpp)
target_link_libraries(test_glc_planner_core 
                      ${GTEST_BOTH_LIBRARIES} 
                      glc_planner_core)                      
add_test(gtest_glc_planner_core test_glc_planner_core)
