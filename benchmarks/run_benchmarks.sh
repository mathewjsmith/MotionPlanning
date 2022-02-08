#!/bin/bash
for filename in instances/new/rand*n032*d16x16*c0*; do
     timeout -s SIGINT 10m julia 'benchmarks/benchmark_cbs.jl' $filename
    #  julia -p 64 'benchmarks/benchmark_constraintga.jl' $filename
done
