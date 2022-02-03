# activate the MotionPlanning package. Assumes we are in the root directory of the project.

using Pkg
Pkg.activate(".")

using MotionPlanning.Model
using MotionPlanning.Benchmarks
using MotionPlanning.IO
using MotionPlanning.SingleRobotPlanning
using MotionPlanning.MultiRobotPlanning.Metrics

using DataStructures
using Glob


# this setting makes it possible to send an InterruptException from a bash script.

Base.exit_on_sigint(false)


# load the instance from the file sent from the bash script.

filename = ARGS[1]

inst = readinstance(filename)

# benchmark parameters.

algorithm = "IndvPaths"

timeout = 600 # seconds

# run the benchmark.

try
    print("benchmarking $algorithm on $(inst.name): ")

    result = @timed plantosolution([ astar(r.pos, r.target, inst) for r in inst.robots ])

    if !isnothing(result.value)
        solution = result.value
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
        isnothing(solution) ? nothing : makespan(solution, :solution),      # makespan
        isnothing(solution) ? nothing : totaldist(solution, :solution),     # totalmoves
        1,                                                # max degree of coupling
        1                                                 # avg degree of coupling
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
        nothing              # mean degree of coupling
    )

    writebenchmark(bm, "benchmarks/results")

    println(e)
end
