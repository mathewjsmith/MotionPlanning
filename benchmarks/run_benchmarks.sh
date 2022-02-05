#!/bin/bash
for filename in instances/new/rand*n032*d8x8*; do
    timeout -s SIGINT 10m julia -p 7 'benchmarks/benchmark_constraintga.jl' $filename
done
