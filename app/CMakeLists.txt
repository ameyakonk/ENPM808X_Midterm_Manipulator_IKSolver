add_executable(shell-app main.cpp Manipulator.cpp Inverse_kinematics.cpp Forward_kinematics.cpp)
find_package(PythonLibs 3.8)
#find_package(Python3 REQUIRED PATHS /usr/lib/x86_64-linux-gnu/)
target_include_directories(shell-app PRIVATE ${PYTHON_INCLUDE_DIR})
target_link_libraries(shell-app ${PYTHON_LIBRARY})
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/eigen-git-mirror
    
)
