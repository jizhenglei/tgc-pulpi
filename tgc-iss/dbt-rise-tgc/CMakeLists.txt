cmake_minimum_required(VERSION 3.12)
###############################################################################
#
###############################################################################
project(dbt-rise-tgc VERSION 1.0.0)

include(GNUInstallDirs)

find_package(elfio QUIET)
find_package(Boost COMPONENTS coroutine)

if(WITH_LLVM)
    if(DEFINED ENV{LLVM_HOME})
        find_path (LLVM_DIR LLVM-Config.cmake $ENV{LLVM_HOME}/lib/cmake/llvm)
    endif(DEFINED ENV{LLVM_HOME})
    find_package(LLVM REQUIRED CONFIG)
    message(STATUS "Found LLVM ${LLVM_PACKAGE_VERSION}")
    message(STATUS "Using LLVMConfig.cmake in: ${LLVM_DIR}")
    llvm_map_components_to_libnames(llvm_libs support core mcjit x86codegen x86asmparser)
endif()

#Mac needed variables (adapt for your needs - http://www.cmake.org/Wiki/CMake_RPATH_handling#Mac_OS_X_and_the_RPATH)
#set(CMAKE_MACOSX_RPATH ON)
#set(CMAKE_SKIP_BUILD_RPATH FALSE)
#set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
#set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
#set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_subdirectory(softfloat)

# library files
FILE(GLOB TGC_SOURCES    ${CMAKE_CURRENT_SOURCE_DIR}/src/iss/*.cpp) 
FILE(GLOB TGC_VM_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/vm/interp/vm_*.cpp)

set(LIB_SOURCES 
    src/vm/fp_functions.cpp
    src/plugin/instruction_count.cpp
    
    ${TGC_SOURCES}
    ${TGC_VM_SOURCES}
)
if(TARGET RapidJSON)
    list(APPEND LIB_SOURCES src/plugin/cycle_estimate.cpp src/plugin/pctrace.cpp)
endif()

if(WITH_LLVM)
	FILE(GLOB TGC_LLVM_SOURCES
	    ${CMAKE_CURRENT_SOURCE_DIR}/src/vm/llvm/vm_*.cpp
	)
	list(APPEND LIB_SOURCES ${TGC_LLVM_SOURCES})
endif()

if(WITH_TCC)
	FILE(GLOB TGC_TCC_SOURCES
	    ${CMAKE_CURRENT_SOURCE_DIR}/src/vm/tcc/vm_*.cpp
	)
	list(APPEND LIB_SOURCES ${TGC_TCC_SOURCES})
endif()

# Define the library
add_library(${PROJECT_NAME} ${LIB_SOURCES})
# list code gen dependencies
if(TARGET ${CORE_NAME}_cpp)
    add_dependencies(${PROJECT_NAME} ${CORE_NAME}_cpp)
endif()

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
     target_compile_options(${PROJECT_NAME} PRIVATE -Wno-shift-count-overflow)
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    target_compile_options(${PROJECT_NAME} PRIVATE /wd4293)
endif()
target_include_directories(${PROJECT_NAME} PUBLIC incl)
target_link_libraries(${PROJECT_NAME} PUBLIC softfloat scc-util jsoncpp Boost::coroutine)
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    target_link_libraries(${PROJECT_NAME} PUBLIC -Wl,--whole-archive dbt-rise-core -Wl,--no-whole-archive)
else()
    target_link_libraries(${PROJECT_NAME} PUBLIC dbt-rise-core)
endif()
if(TARGET CONAN_PKG::elfio)
    target_link_libraries(${PROJECT_NAME} PUBLIC CONAN_PKG::elfio)
elseif(TARGET elfio::elfio)
    target_link_libraries(${PROJECT_NAME} PUBLIC elfio::elfio)
else()
    message(FATAL_ERROR "No elfio library found, maybe a find_package() call is missing")
endif()
if(TARGET RapidJSON)
    target_link_libraries(${PROJECT_NAME} PUBLIC RapidJSON)
endif()


set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION ${PROJECT_VERSION}
  FRAMEWORK FALSE
)
install(TARGETS ${PROJECT_NAME} COMPONENT ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets            # for downstream dependencies
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}  # static lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}  # binaries
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}  # shared lib
  FRAMEWORK DESTINATION ${CMAKE_INSTALL_LIBDIR} # for mac
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME} # headers for mac (note the different component -> different package)
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}             # headers
)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/incl/iss COMPONENT ${PROJECT_NAME}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} # target directory
        FILES_MATCHING # install only matched files
        PATTERN "*.h" # select header files
        )
###############################################################################
#
###############################################################################
project(tgc-sim)
find_package(Boost COMPONENTS program_options thread REQUIRED)

add_executable(${PROJECT_NAME} src/main.cpp)
foreach(F IN LISTS TGC_SOURCES)
    string(REGEX REPLACE  ".*/([^/]*)\.cpp"  "\\1" CORE_NAME_LC ${F})
    string(TOUPPER ${CORE_NAME_LC} CORE_NAME)
    target_compile_definitions(${PROJECT_NAME} PRIVATE CORE_${CORE_NAME})
endforeach()

if(WITH_LLVM)
    target_compile_definitions(${PROJECT_NAME} PRIVATE WITH_LLVM)
    target_link_libraries(${PROJECT_NAME} PUBLIC ${llvm_libs})
endif()
# Links the target exe against the libraries
target_link_libraries(${PROJECT_NAME} PUBLIC dbt-rise-tgc)
if(TARGET Boost::program_options)
    target_link_libraries(${PROJECT_NAME} PUBLIC Boost::program_options)
else()
    target_link_libraries(${PROJECT_NAME} PUBLIC ${BOOST_program_options_LIBRARY})
endif()
target_link_libraries(${PROJECT_NAME} PUBLIC ${CMAKE_DL_LIBS})
if (Tcmalloc_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC ${Tcmalloc_LIBRARIES})
endif(Tcmalloc_FOUND)

install(TARGETS tgc-sim
  EXPORT ${PROJECT_NAME}Targets            # for downstream dependencies
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}  # static lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}  # binaries
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}  # shared lib
  FRAMEWORK DESTINATION ${CMAKE_INSTALL_LIBDIR} # for mac
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}  # headers for mac (note the different component -> different package)
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}             # headers
)
###############################################################################
#
###############################################################################
project(dbt-rise-tgc_sc VERSION 1.0.0)

include(SystemCPackage)
if(SystemC_FOUND)
    add_library(${PROJECT_NAME} src/sysc/core_complex.cpp)
    target_compile_definitions(${PROJECT_NAME} PUBLIC WITH_SYSTEMC)
    target_compile_definitions(${PROJECT_NAME} PRIVATE CORE_${CORE_NAME})
    foreach(F IN LISTS TGC_SOURCES)
        string(REGEX REPLACE  ".*/([^/]*)\.cpp"  "\\1" CORE_NAME_LC ${F})
        string(TOUPPER ${CORE_NAME_LC} CORE_NAME)
        target_compile_definitions(${PROJECT_NAME} PRIVATE CORE_${CORE_NAME})
    endforeach()
    target_link_libraries(${PROJECT_NAME} PUBLIC dbt-rise-tgc scc)
    if(WITH_LLVM)
        target_link_libraries(${PROJECT_NAME} PUBLIC ${llvm_libs})
    endif()
    
	set(LIB_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/incl/sysc/core_complex.h)
    set_target_properties(${PROJECT_NAME} PROPERTIES
      VERSION ${PROJECT_VERSION}
      FRAMEWORK FALSE
      PUBLIC_HEADER "${LIB_HEADERS}" # specify the public headers
    )
    install(TARGETS ${PROJECT_NAME} COMPONENT ${PROJECT_NAME}
	  EXPORT ${PROJECT_NAME}Targets            # for downstream dependencies
	  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}  # static lib
	  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}  # binaries
	  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}  # shared lib
	  FRAMEWORK DESTINATION ${CMAKE_INSTALL_LIBDIR} # for mac
	  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/sysc   # headers for mac (note the different component -> different package)
	  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}             # headers
	)    
endif()

