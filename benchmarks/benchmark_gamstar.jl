# activate the MotionPlanning package. Assumes we are in the root directory of the project.

using Distributed

@everywhere begin
	using Pkg
	Pkg.activate(".")
end

@everywhere using MotionPlanning.GeneticAlgorithms.ConstraintGA2

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

inst.dims = (8, 8)


# load the solution found by the ConstraintGA.

preprocessed = begin
    garesult = readbenchmark("benchmarks/results/$(inst.name)_ConstraintGA3")

    if isnothing(garesult.solution)
        nothing
    else
        solutiontoplan(garesult.solution)
    end
end


# benchmark parameters.

algorithm = "GA-MStar"

objective = MakeSpan()

heuristic = MaxDist()

timeout = 600 # seconds

# run the algorithm on an easy instance to eliminate overhead from compiling

robots = [
    Robot(1, (0, 0), (0, 1)),
    Robot(2, (1, 1), (1, 0))
]

dead_inst = MRMPInstance("test", robots, Vector{Obstacle}())

mstar(dead_inst, objective, heuristic)

# run the benchmark.

try
    print("benchmarking $algorithm on $(inst.name): ")

    result = @timed mstar(inst, objective, heuristic; initialplan=preprocessed)

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
        length(inst.robots),                                                # nrobots
        solution,                                                           # solution
        result.time,                                                        # time
        nothing,                                                            # ϵ
        isnothing(result.value) ? nothing : makespan(solution, :solution),  # makespan
        isnothing(result.value) ? nothing : totaldist(solution, :solution), # totalmoves
        max,                                                                # max degree of coupling
        mean,                                                               # avg degree of coupling
        nothing
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
        nothing,             # makespan
        nothing,             # totalmoves
        nothing,             # max degree of coupling
        nothing,             # mean degree of coupling
        nothing
    )

    writebenchmark(bm, "benchmarks/results")

    println(e)
end
