set(GTEST_SHUFFLE 1)

add_executable(
    cpp-test
    main.cpp
    IK_test.cpp
    FK_test.cpp
    ../app/Inverse_kinematics.cpp
    ../app/Forward_kinematics.cpp
)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/eigen-git-mirror
    
)
target_include_directories(cpp-test PUBLIC ../vendor/googletest/googletest/include 
                                           ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(cpp-test PUBLIC gtest)
