
include_directories(${PROJECT_SOURCE_DIR}/glc/include)

add_executable(shortest-path-demo shortest-path-demo.cpp)
target_link_libraries(shortest-path-demo glc_planner_core glc_logging)

add_executable(nonholonomic-car-demo nonholonomic-car-demo.cpp)
target_link_libraries(nonholonomic-car-demo glc_planner_core glc_logging)

add_executable(pendulum-swingup-demo pendulum-swingup-demo.cpp)
target_link_libraries(pendulum-swingup-demo glc_planner_core glc_logging)