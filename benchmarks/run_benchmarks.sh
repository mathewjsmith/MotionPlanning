#!/bin/bash
for filename in instances/new/rand*n*{15..25}*d10x10*; do
    timeout -s SIGINT 10m julia 'benchmarks/benchmark_cbs.jl' $filename
done
