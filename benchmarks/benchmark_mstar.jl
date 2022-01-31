# activate the MotionPlanning package. Assumes we are in the root directory of the project.

using Pkg
Pkg.activate(".")

using MotionPlanning.Model
using MotionPlanning.Benchmarks
using MotionPlanning.IO
using MotionPlanning.MultiRobotPlanning.Objectives
using MotionPlanning.MultiRobotPlanning.Heuristics
using MotionPlanning.MultiRobotPlanning.Metrics
using MotionPlanning.MultiRobotPlanning.MStar

using DataStructures
using Glob


# this setting makes it possible to send an InterruptException from a bash script.

Base.exit_on_sigint(false)


# load the instance from the file sent from the bash script.

filename = ARGS[1]

inst = readinstance(filename)

# benchmark parameters.

algorithm = "MStar"

objective = MakeSpan()

heuristic = MaxDist()

timeout = 300 # seconds


# run the algorithm on an easy instance to eliminate overhead from compiling

robots = [
    Robot(1, (0, 0), (0, 1)),
    Robot(2, (1, 1), (1, 0))
]

deadinst = MRMPInstance("test", robots, Vector{Obstacle}())

mstar(deadinst, objective, heuristic)

# run the benchmark.

try
    print("benchmarking $algorithm on $(inst.name): ")

    result = @timed mstar(inst, objective, heuristic)

    if !isnothing(result.value)
        solution, G, path = result.value
        min, max, mean    = κ(G, path)
    else
        solution = nothing
    end


    bm = Benchmark(
        algorithm,
        inst.name, 
        inst.dims,
        length(inst.robots),                                             # nrobots
        solution,                                                        # solution
        result.time,                                                     # time
        nothing,                                                         # ϵ
        isnothing(solution) ? 0 : makespan(solution, :solution),         # makespan
        isnothing(solution) ? 0 : totaldist(solution, :solution),        # totalmoves
        max                                                              # degree of coupling
    )

    writebenchmark(bm, "benchmarks/results")

    println("completed in $(bm.time) seconds with a makespan of $(bm.makespan) and $(bm.totalmoves) total moves.")
    println("$κ --- min: $min, max: $max, mean: $mean")
catch e
    bm = Benchmark(
        algorithm,
        inst.name, 
        inst.dims,
        length(inst.robots), # nrobots
        nothing,             # solution
        timeout,             # time
        nothing,             # ϵ
        0,                   # makespan
        0,                   # totalmoves
        -1                   # degree of coupling
    )

    writebenchmark(bm, "benchmarks/results")

    println(e)
end
