# CMakeList.txt : CMake project for ArtificialVison, include source and define
# project specific logic here.
#

# Add source to this project's executable.
add_executable (ArtificialVison "ArtificialVison.cpp" "ArtificialVison.h")

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET ArtificialVison PROPERTY CXX_STANDARD 20)
endif()

target_compile_features(ArtificialVison PRIVATE cxx_std_20)

if (MSVC)
  target_compile_options(ArtificialVison PRIVATE /W4 /permissive-)
else()
  target_compile_options(ArtificialVison PRIVATE -Wall -Wextra -Wpedantic)
endif()
