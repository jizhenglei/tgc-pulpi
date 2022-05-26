macro(setup_conan)
  set(options Release Debug RelWithDebInfo TARGETS)
  cmake_parse_arguments(MARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )
  
  if(MARGS_Release)
  	set (CONAN_BUILD_TYPE Release)
  elseif(MARGS_Debug)
    set (CONAN_BUILD_TYPE Debug)
  elseif(MARGS_RelWithDebInfo)
    set (CONAN_BUILD_TYPE RelWithDebInfo) 
  endif()

  find_program(conan conan)
  if(NOT EXISTS ${conan})
    message(FATAL_ERROR "Conan is required. Please see README.md")
    return()
  endif()
  execute_process(COMMAND ${conan} --version
                  OUTPUT_VARIABLE CONAN_VERSION_OUTPUT)
  string(REGEX MATCHALL "[0-9.]+" CONAN_VERSION ${CONAN_VERSION_OUTPUT})
 
  #['Visual Studio', 'apple-clang', 'clang', 'gcc', 'intel', 'mcst-lcc', 'msvc', 'qcc', 'sun-cc']
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(CONAN_COMPILER -s compiler=gcc)
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    set(CONAN_COMPILER -s compiler=apple-clang)
  else()
    message(FATAL_ERROR "Unknown compiler: ${CMAKE_CXX_COMPILER_ID}")
  endif()

  if (NOT "$ENV{CONAN_PROFILE_NAME}" STREQUAL "")
    set(CONAN_PROFILE "-pr" "$ENV{CONAN_PROFILE_NAME}" CACHE INTERNAL "Copied from environment variable")
  else()
    set(CONAN_PROFILE "-pr" "default" CACHE INTERNAL "Copied from environment variable")
  endif()

  set(conanfile ${CMAKE_SOURCE_DIR}/conanfile.txt)
  set(conanfile_cmake ${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
  set(conanfile_cmake_paths ${CMAKE_BINARY_DIR}/conan_paths.cmake)
  
  if(NOT CONAN_BUILD_TYPE)
    if("${CMAKE_BUILD_TYPE}" STREQUAL "")
	  set(CONAN_BUILD_TYPE -s build_type=Debug)
    elseif("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
	  set(CONAN_BUILD_TYPE -s build_type=Release)
    else()
	  set(CONAN_BUILD_TYPE -s build_type=${CMAKE_BUILD_TYPE})
    endif()
  endif()

  if(NOT ${CMAKE_CXX_STANDARD})
    set(CONAN_CXXSTD "")
  else()
    if (CONAN_VERSION VERSION_GREATER_EQUAL 1.36)
      set(CONAN_CXXSTD -s compiler.cppstd=${CMAKE_CXX_STANDARD})
    else()
      set(CONAN_CXXSTD -s cppstd=${CMAKE_CXX_STANDARD})
    endif()
  endif()

  #message("Running conan as '${conan} install ${CMAKE_SOURCE_DIR} --build=missing ${CONAN_PROFILE} ${CONAN_COMPILER} ${CONAN_CXXSTD} ${CONAN_BUILD_TYPE}'")
  execute_process(
    COMMAND ${conan} install ${CMAKE_SOURCE_DIR} --build=missing ${CONAN_PROFILE} ${CONAN_COMPILER} ${CONAN_CXXSTD} ${CONAN_BUILD_TYPE}
    RESULT_VARIABLE return_code)
  if(NOT ${return_code} EQUAL 0)
    message(FATAL_ERROR "conan install command failed.")
  endif()

  if(EXISTS "${conanfile_cmake_paths}")
  	include(${conanfile_cmake_paths})
  elseif(EXISTS "${conanfile_cmake}")
    include(${conanfile_cmake})
    if( MARGS_TARGETS)
  	  conan_basic_setup(TARGETS)
    else()
      conan_basic_setup()
    endif()
  endif()
endmacro()
