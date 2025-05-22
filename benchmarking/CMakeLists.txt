include(../cmake/testing_utils.cmake)

SETUP_BENCHMARK_ENV()

if(benchmark_FOUND)

  # MY benchmarks
  set( BENCHMARKS_DIR benchmarks )
  set( BENCHMARKS
    my_dummy_bechmarks
    my_benchmark_dev2
    my_benchmark_dev3
    )

  ADD_BENCHMARKS(dte3607::physengine ${BENCHMARKS_DIR} BENCHMARKS)

endif(benchmark_FOUND)
