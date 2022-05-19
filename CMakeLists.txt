project(ded)
cmake_minimum_required(VERSION 3.22)

macro(print_all_variables)
    message(STATUS "print_all_variables------------------------------------------{")
    get_cmake_property(_variableNames VARIABLES)
    foreach (_variableName ${_variableNames})
        message(STATUS "${_variableName}=${${_variableName}}")
    endforeach()
    message(STATUS "print_all_variables------------------------------------------}")
endmacro()


find_package( Eigen3 REQUIRED )
find_package(X11 REQUIRED)
find_package(glfw3 REQUIRED)



include_directories( "/usr/include/eigen3" )

add_executable( ded main.cpp /usr/src/gl.c )
target_link_libraries( ded X11 glfw3 )

#print_all_variables()c