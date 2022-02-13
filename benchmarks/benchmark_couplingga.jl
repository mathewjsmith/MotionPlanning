# activate the MotionPlanning package. Assumes we are in the root directory of the project.

using Distributed

@everywhere begin
    using Pkg

    Pkg.activate(".")
    Pkg.precompile()
end

@everywhere using MotionPlanning.GeneticAlgorithms.CouplingGA3

using MotionPlanning.Model
using MotionPlanning.Benchmarks
using MotionPlanning.IO
using MotionPlanning.MultiRobotPlanning.Metrics

using DataStructures
using Glob


# this setting makes it possible to send an InterruptException from a bash script.

Base.exit_on_sigint(false)


# load the instance from the file sent from the bash script.

filename = ARGS[1]

inst = readinstance(filename)

inst.dims = (8, 8)

# benchmark parameters.

algorithm = "CouplingGA"

timeout = 600 # seconds

n = length(inst.robots)

params = Params(128, 4, 0.9, 0.01, ceil(Int, log2(n)))


# run the benchmark.

try
    print("benchmarking $algorithm on $(inst.name): ")

    result = @timed evolve(inst, params; maxgens=4096)

    if !isnothing(result.value)
        solution = result.value
    else
        solution = nothing
    end

    kappa  = ceil(Int, log2(n))

    bm = Benchmark(
        algorithm,
        inst.name, 
        inst.dims,
        length(inst.robots),                                                # nrobots
        solution,                                                           # solution
        result.time,                                                        # time
        nothing,                                                            # ϵ
        isnothing(solution) ? nothing : makespan(solution, :solution),      # makespan
        isnothing(solution) ? nothing : totaldist(solution, :solution),     # totalmoves
        kappa,                                                              # max degree of coupling
        nothing,                                                            # avg degree of coupling
        nothing                                                               # ga stats
    )

    writebenchmark(bm, "benchmarks/results")

    println("completed in $(bm.time) seconds with a makespan of $(bm.makespan) and $(bm.totalmoves) total moves.")
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
        nothing              # ga stats
    )

    writebenchmark(bm, "benchmarks/results")

    println(e)
end
