add_executable(wombat_tests
        ResultTest.cpp
        DeviceTypesTest.cpp
        DeviceControllerTest.cpp
        SpiMockTest.cpp
        DataPublisherTest.cpp
        ${CMAKE_SOURCE_DIR}/${TEST_SPI_SOURCE}
)

target_include_directories(wombat_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(wombat_tests PRIVATE
        wombat_lib
        GTest::gtest_main
        GTest::gmock
)

include(GoogleTest)
gtest_discover_tests(wombat_tests)

# Transport contract: the real binary refuses to start on a mismatch (exit 78).
add_test(NAME reader_contract_smoke
         COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/contract_smoke.sh $<TARGET_FILE:stm32_data_reader>)
