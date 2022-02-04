#!/bin/bash
for filename in instances/new/rand*n0{10..44}*d8x8*; do
    timeout -s SIGINT 10m julia 'benchmarks/benchmark_shadoks.jl' $filename
done
