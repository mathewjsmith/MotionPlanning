# activate the MotionPlanning package. Assumes we are in the root directory of the project.

using Distributed

@everywhere begin
    using Pkg

    Pkg.activate(".")
    Pkg.precompile()

end

@everywhere using MotionPlanning.GeneticAlgorithms.ConstraintGA2

# @everywhere begin
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

    algorithm = "ConstraintGA"

    timeout = 600 # seconds

    params = Params(32, 4, 0.95, 0.01, 32)


    # run the benchmark.

    try
        print("benchmarking $algorithm on $(inst.name): ")

        result = @timed evolve(inst, params; maxgens=256)

        if !isnothing(result.value)
            solution, chrom = result.value
            kappa  = sum([ nconstraints(g) for g in chrom ])
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
            kappa,                                                              # max degree of coupling
            nothing                                                             # avg degree of coupling
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
# end