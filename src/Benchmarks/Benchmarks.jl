module Benchmarks

include("Generators.jl")

using MotionPlanning.Model
using MotionPlanning.MultiRobotPlanning.Graph
using MotionPlanning.GeneticAlgorithms.ConstraintGA2

using JSON2

export
    Benchmark,
    writebenchmark, 
    readbenchmark,
    Generators


struct Benchmark
    algorithm    :: String
    instancename :: String
    dims         :: Tuple{Int64, Int64}
    nrobots      :: Int64
    solution     :: Union{Nothing, Array{Array{Pos, 1}, 1}}
    time         :: Float64
    ϵ            :: Union{Int64, Nothing}
    makespan     :: Union{Int64, Nothing}
    totalmoves   :: Union{Int64, Nothing}
    maxkappa     :: Union{Int64, Nothing}
    avgkappa     :: Union{Float64, Nothing}
    gastats      :: Union{Vector{GAStats}, Nothing}
end


function writebenchmark(benchmark::Benchmark, folder::String)
    json = JSON2.write(benchmark)
    open("$folder/$(benchmark.instancename)_$(benchmark.algorithm)", "w") do file
        write(file, json)
    end
end


function readbenchmark(filename::String)
    open(filename) do file
        JSON2.read(file, Benchmark)
    end
end


end