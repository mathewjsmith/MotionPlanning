#!/bin/bash
for filename in instances/new/rand*n007*d8x8*; do
    timeout -s SIGINT 10m julia 'benchmarks/benchmark_astar.jl' $filename
done
