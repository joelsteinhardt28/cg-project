# CMakeLists.txt loosely based on https://github.com/nmwsharp/polyscope/blob/v1.3.0/examples/demo-app/CMakeLists.txt

cmake_minimum_required(VERSION 3.14)

project(cg2-exercises LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Maybe stop from CMAKEing in the wrong place
if (CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR)
    message(FATAL_ERROR "Source and build directories cannot be the same. Go use the /build directory.")
endif()

### Configure output locations
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

### Compiler options
set( CMAKE_EXPORT_COMPILE_COMMANDS 1 ) # Emit a compile flags file to support completion engines 

if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  # using Clang (linux or apple) or GCC
  message("Using clang/gcc compiler flags")
  SET(BASE_CXX_FLAGS "-std=c++17 -Wall -Wextra -g3 -march=native")
  SET(DISABLED_WARNINGS " -Wno-unused-parameter -Wno-unused-variable -Wno-unused-function -Wno-deprecated-declarations -Wno-missing-braces")
  SET(TRACE_INCLUDES " -H -Wno-error=unused-command-line-argument")

  if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    message("Setting clang-specific options")
    SET(BASE_CXX_FLAGS "${BASE_CXX_FLAGS} -ferror-limit=5 -fcolor-diagnostics")
    SET(CMAKE_CXX_FLAGS_DEBUG          "-fsanitize=address -fno-limit-debug-info")
  elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    SET(BASE_CXX_FLAGS "${BASE_CXX_FLAGS} -fmax-errors=5")
    message("Setting gcc-specific options")
    SET(DISABLED_WARNINGS "${DISABLED_WARNINGS} -Wno-maybe-uninitialized -Wno-format-zero-length -Wno-unused-but-set-parameter -Wno-unused-but-set-variable")
  endif()

  SET(CMAKE_CXX_FLAGS "${BASE_CXX_FLAGS} ${DISABLED_WARNINGS}")
  #SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TRACE_INCLUDES}") # uncomment if you need to track down where something is getting included from
  SET(CMAKE_CXX_FLAGS_DEBUG          "${CMAKE_CXX_FLAGS_DEBUG} -g3")
  # IMPORTANT: Do NOT enable -ffast-math for Release or any build type!
  # -ffast-math breaks IEEE 754 compliance and floating-point/integer semantic guarantees,
  # which silently destroys exact 256-bit geometric predicates in typed-geometry and integer-plane-geometry.
  if (CMAKE_CXX_FLAGS_RELEASE MATCHES "-ffast-math" OR CMAKE_CXX_FLAGS MATCHES "-ffast-math")
    message(FATAL_ERROR "-ffast-math must NOT be enabled as it silently destroys exact 256-bit geometric predicates.")
  endif()

  SET(CMAKE_CXX_FLAGS_RELEASE        "-march=native -O3 -DNDEBUG")
  SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  # using Visual Studio C++
  message("Using Visual Studio compiler flags")
  set(BASE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4 /arch:AVX2")
  set(BASE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP") # parallel build
  SET(DISABLED_WARNINGS "${DISABLED_WARNINGS} /wd\"4267\"")  # ignore conversion to smaller type (fires more aggressively than the gcc version, which is annoying)
  SET(DISABLED_WARNINGS "${DISABLED_WARNINGS} /wd\"4244\"")  # ignore conversion to smaller type (fires more aggressively than the gcc version, which is annoying)
  SET(DISABLED_WARNINGS "${DISABLED_WARNINGS} /wd\"4305\"")  # ignore truncation on initialization
  SET(CMAKE_CXX_FLAGS "${BASE_CXX_FLAGS} ${DISABLED_WARNINGS}")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd")
  add_compile_definitions(_USE_MATH_DEFINES)

  add_definitions(/D "_CRT_SECURE_NO_WARNINGS")
  add_definitions (-DNOMINMAX)
else()
  # unrecognized
  message( FATAL_ERROR "Unrecognized compiler [${CMAKE_CXX_COMPILER_ID}]" )
endif()

include(FetchContent)

FetchContent_Declare(
  portable-file-dialogs
  GIT_REPOSITORY https://github.com/samhocevar/portable-file-dialogs.git
  GIT_TAG 0.1.0
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE)
FetchContent_MakeAvailable(portable-file-dialogs)

FetchContent_Declare(
  polyscope
  GIT_REPOSITORY https://github.com/nmwsharp/polyscope.git
  GIT_TAG v2.4.0
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE)
FetchContent_MakeAvailable(polyscope)

# Disable PMP's visualization, tests, and examples to prevent dependency collisions
set(PMP_BUILD_VIEWERS OFF CACHE INTERNAL "")
set(PMP_BUILD_VIS OFF CACHE INTERNAL "") # For compatibility with older PMP commits
set(PMP_BUILD_APPS OFF CACHE INTERNAL "")
set(PMP_BUILD_EXAMPLES OFF CACHE INTERNAL "")
set(PMP_BUILD_TESTS OFF CACHE INTERNAL "")
set(PMP_BUILD_DOCS OFF CACHE INTERNAL "")

FetchContent_Declare(
  pmp
  GIT_REPOSITORY https://github.com/pmp-library/pmp-library.git
  GIT_TAG 3.0.0
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE)
FetchContent_MakeAvailable(pmp)

# Force TG_ENABLE_FIXED_INT to ON since our CPU supports it and integer-plane-geometry requires it
set(TG_ENABLE_FIXED_INT ON CACHE BOOL "Enable typed-geometry fixed_int feature" FORCE)

# Disable mold linker in clean-core to avoid linking errors if mold is not installed
set(CC_LINKER_MOLD OFF CACHE BOOL "Disable mold linker" FORCE)

FetchContent_Declare(
  clean-core
  GIT_REPOSITORY https://github.com/project-arcana/clean-core.git
  GIT_TAG develop
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE)
FetchContent_MakeAvailable(clean-core)
set_target_properties(clean-core PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES $<TARGET_PROPERTY:clean-core,INTERFACE_INCLUDE_DIRECTORIES>)

FetchContent_Declare(
  typed-geometry
  GIT_REPOSITORY https://github.com/project-arcana/typed-geometry.git
  GIT_TAG develop
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE)
FetchContent_MakeAvailable(typed-geometry)
set_target_properties(typed-geometry PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES $<TARGET_PROPERTY:typed-geometry,INTERFACE_INCLUDE_DIRECTORIES>)

find_package(OpenMP REQUIRED)

# Create an executable
add_executable(project src/main.cpp src/io.cpp src/gui.cpp src/mesh_utils.cpp src/kernel_gen.cpp src/toolbox.cpp)

if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  target_compile_options(project PRIVATE -Werror)
endif()

# Add header-only dependencies
target_include_directories(project PRIVATE include)
target_include_directories(project PRIVATE src)
target_include_directories(project PRIVATE "${polyscope_SOURCE_DIR}/deps/args")
target_include_directories(project PRIVATE "${polyscope_SOURCE_DIR}/deps/json/include")
target_include_directories(project PRIVATE "${pmp_SOURCE_DIR}/src")

# Link to dependencies
target_link_libraries(project PRIVATE 
  polyscope 
  portable_file_dialogs 
  pmp 
  clean-core 
  typed-geometry
  OpenMP::OpenMP_CXX
)
