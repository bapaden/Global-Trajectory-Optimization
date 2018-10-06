include_directories(./include)

#Level-0 of local dependency graph
add_library(glc_math src/glc_math.cpp)
add_library(glc_interpolation src/glc_interpolation.cpp)
add_library(glc_parameters src/glc_parameters.cpp)

#Level-1 of local dependency graph
add_library(glc_node src/glc_node.cpp)
target_link_libraries(glc_node
                      glc_interpolation)
                      
add_library(glc_state_equivalence_class src/glc_state_equivalence_class.cpp)
target_link_libraries(glc_state_equivalence_class
                      glc_node
                      glc_math)

add_library(glc_interface src/glc_interface.cpp)
target_link_libraries(glc_interface
                      glc_interpolation)
                      
#Level-2 of local dependency graph
add_library(glc_logging src/glc_logging.cpp)
target_link_libraries(glc_logging 
                      glc_interpolation
                      glc_state_equivalence_class)
                      
add_library(glc_numerical_integration src/glc_numerical_integration.cpp)
target_link_libraries(glc_numerical_integration
                      glc_interpolation
                      glc_interface)

#Level-3 of local dependency graph                      
add_library(glc_planner_core src/glc_planner_core.cpp)
target_link_libraries(glc_planner_core
                      glc_numerical_integration
                      glc_logging)


# Install library
install(TARGETS glc_planner_core glc_numerical_integration glc_logging glc_interface glc_state_equivalence_class glc_node glc_math glc_parameters glc_interpolation
  EXPORT ${PROJECT_EXPORT}
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT stlib
  COMPONENT dev)
  
# Set HEADERS variable
file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)
  
# # Set SOURCES variable
# file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

# Install headers
install(FILES ${HEADERS}
  DESTINATION "${INSTALL_INCLUDE_DIR}/${LIBRARY_FOLDER}" )

#include(${CMAKE_SOURCE_DIR}/cmake/LibraryConfig.cmake)
