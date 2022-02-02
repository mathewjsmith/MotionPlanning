#!/bin/bash
for filename in instances/new/rand*n0{10..16}*d8x8*; do
    timeout -s SIGINT 10m julia 'benchmarks/benchmark_cbs.jl' $filename
done
