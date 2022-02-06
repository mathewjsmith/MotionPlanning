#!/bin/bash
for filename in instances/new/rand*n00{4..9}*d8x8*; do
     timeout -s SIGINT 10m julia 'benchmarks/benchmark_gacbs.jl' $filename
    #  julia -p 64 'benchmarks/benchmark_constraintga.jl' $filename
done
