#!/bin/bash
for filename in instances/new/rand*n0{10..44}*d8x8*; do
     # timeout -s SIGINT 12m julia -p 64 'benchmarks/benchmark_constraintga.jl' $filename
     julia -p 64 'benchmarks/benchmark_constraintga.jl' $filename
done
