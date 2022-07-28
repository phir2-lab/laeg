macro(Setup)
    message(STATUS "Running Setup...")

    IF(NOT CMAKE_BUILD_TYPE)
        SET(CMAKE_BUILD_TYPE Release)
    ENDIF()

    string(TOLOWER "${CMAKE_BUILD_TYPE}" BUILD_TYPE)

	find_package(
	  catkin REQUIRED COMPONENTS 
	  roscpp
	  roslib
	  laser_geometry
	  tf
	)
	include_directories(${catkin_LIBRARY_DIRS})
	include_directories(${catkin_INCLUDE_DIRS})

    # CHECK C++14 SUPPORT
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
    if(COMPILER_SUPPORTS_CXX14)
       set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
       add_definitions(-DCOMPILEDWITHC14)
       message(STATUS "Using flag -std=c++14.")
    else()
       message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
    endif()

    # FIND GLUT
    find_package(GLUT REQUIRED)
    include_directories(${GLUT_INCLUDE_DIRS})
    link_directories(${GLUT_LIBRARY_DIRS})
    add_definitions(${GLUT_DEFINITIONS})
    if(NOT GLUT_FOUND)
        message(ERROR "GLUT not found!")
    endif(NOT GLUT_FOUND)

    # FIND OPENGL
    find_package(OpenGL REQUIRED)
    include_directories(${OpenGL_INCLUDE_DIRS})
    link_directories(${OpenGL_LIBRARY_DIRS})
    add_definitions(${OpenGL_DEFINITIONS})
    if(NOT OPENGL_FOUND)
        message(ERROR "OPENGL not found!")
    endif(NOT OPENGL_FOUND)

    #FIND REQUIRED PACKAGE
    find_package(FREEGLUT QUIET)
    find_package(Threads REQUIRED)

endmacro(Setup)

