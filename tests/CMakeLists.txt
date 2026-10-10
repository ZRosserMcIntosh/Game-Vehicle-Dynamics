set(ORD_TEST_SOURCES
  test_main.cpp
  test_tire.cpp
  test_drivetrain.cpp
  test_vehicle.cpp
  test_assists.cpp
  test_benchmarks.cpp
)
add_executable(ord_tests ${ORD_TEST_SOURCES})
target_link_libraries(ord_tests PRIVATE ord_scenarios)
target_compile_options(ord_tests PRIVATE ${ORD_WARNINGS})

# The runner prints per-case results; pass a substring to run a subset: ord_tests "drivetrain:"
add_test(NAME ord_tests COMMAND ord_tests)
