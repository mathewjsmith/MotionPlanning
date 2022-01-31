module Benchmarks

include("Generators.jl")

using MotionPlanning.Model
using MotionPlanning.MultiRobotPlanning.Graph

using JSON2
using StatsBase

export
    Benchmark,
    writebenchmark, 
    readbenchmark,
    random,
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
    kappa        :: Union{Int64, Nothing}
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


"""
Generates a random MRMP instance based on the specified parameters.
"""
function random(nrobots, nobstacles, width, height, id)
    cells = [ (x, y) for x in 0 : width - 1, y in 0 : height - 1 ]

    positions = sample(cells, nrobots; replace = false)

    targets = sample(cells, nrobots; replace = false)

    robots = [
        Robot(id, pos, target)
        for (id, (pos, target)) in enumerate(zip(positions, targets))
    ]

    obstacles = [
        Obstacle(id, pos)
        for (id, pos) in enumerate(sample(setdiff(cells, positions, targets), nobstacles; replace = false))
    ]

    MRMPInstance("random_$(nrobots)_$(nobstacles)_$(width)x$(height)_$id", robots, obstacles)
end


end