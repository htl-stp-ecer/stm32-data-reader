add_executable(wombat_tests
        ResultTest.cpp
        DeviceTypesTest.cpp
        DeviceControllerTest.cpp
        SpiMockTest.cpp
        DataPublisherTest.cpp
)

target_include_directories(wombat_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(wombat_tests PRIVATE
        wombat_lib
        GTest::gtest_main
        GTest::gmock
)

include(GoogleTest)
gtest_discover_tests(wombat_tests)
