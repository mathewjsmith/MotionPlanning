#!/bin/bash
for filename in instances/new/rand*n*{4..44}*d8x8*; do
    timeout -s SIGINT 10m julia 'benchmarks/benchmark_indvpaths.jl' $filename
done
