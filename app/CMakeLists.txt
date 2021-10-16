add_executable(shell-app main.cpp Manipulator.cpp Inverse_kinematics.cpp Forward_kinematics.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
   # ${CMAKE_SOURCE_DIR}/eigen-git-mirror
    
)
