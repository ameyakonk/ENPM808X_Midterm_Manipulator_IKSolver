add_executable(shell-app main.cpp Manipulator.cpp kinematics.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)
