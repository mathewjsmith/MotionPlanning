module CouplingGA

using MotionPlanning.Collisions
using MotionPlanning.Model
using MotionPlanning.MultiRobotPlanning.CBS
using MotionPlanning.MultiRobotPlanning.Metrics
using MotionPlanning.MultiRobotPlanning.Shadoks
using MotionPlanning.SingleRobotPlanning

using StatsBase
using Statistics

export
    evolve,
    iterevolve,
    Params


const Chromosome = BitVector


struct Params
    popsize  :: Int
    nelites  :: Int
    pcross   :: Float64
    pmut     :: Float64
    kmax     :: Int
end


function iterevolve(inst::MRMPInstance, params::Params, initialsol::Union{Solution, Nothing}=nothing, collthresh::Int=0)
    try
        sol = evolve(inst::MRMPInstance, params, initialsol)    

        ncolls = length(findcollisions(sol, inst))

        if ncolls <= collthresh
            return sol
        end

        iterevolve(inst, params, sol, collthresh)
    catch _
        initialsol
    end
end


function evolve(inst::MRMPInstance, params::Params, initialsol::Union{Solution, Nothing}=nothing; kinit=1)
    chromsize = length(inst.robots)

    solutions = Dict{Chromosome, Union{Solution, Nothing}}()
    fitness   = Dict{Chromosome, Vector{Float64}}()


    if isnothing(initialsol)
        zerosol   = shadoks(inst)
    else
        zerosol = initialsol
    end

    ncolls = length(findcollisions(zerosol, inst))

    population = populate(params.popsize, chromsize, kinit)

    k          = kinit
    generation = 1
    best       = ncolls

    # try
        while k <= params.kmax && generation < 256
            println("generation: $generation, kappa: $k, best: $best")

            # generate the next generation
            if generation > 1
                fitnesses = map(chrom -> fitness[chrom], population)

                selection = select(population, fitnesses, params)

                elites = selectelites(population, fitnesses, params.nelites)

                selection = [ elites; selection ]

                children = reproduce(selection, params)


                population = [ elites; children ]
            end

            # solve instances and find fitnesses for newly discovered chromosomes
            println("solving... ")
            for chrom in population
                if !haskey(solutions, chrom)
                    solution = solve(inst, chrom, zerosol)

                    if isnothing(solution)
                        solutions[chrom] = nothing
                        fitness[chrom] = [0, 0, 0]
                    else
                        ncolls = length(findcollisions(solution, inst))

                        best = min(best, ncolls)

                        solutions[chrom] = solution
                        fitness[chrom]   = f(solution, inst)
                    end
                end
            end
            println("solved.")

            k = maximum(sum(chrom) for chrom in population)

            generation += 1
        end
    # catch _
    #     println("interrupted")

    #     best = first(sort(population, by=chrom -> fitness[chrom], rev=true))

    #     solution[best]
    # end

    best = first(sort(population, by=chrom -> fitness[chrom], rev=true))

    solutions[best]
end


function f(sol::Solution, inst::MRMPInstance)
    ncolls = length(findcollisions(sol, inst))
    sum    = totaldist(sol, :solution)
    max    = makespan(sol, :solution)

    sig(x) = exp(-x) / (1 + exp(-x))

    [ sig(ncolls), sig(sum), sig(max) ]
end

populate(n::Int, l::Int, k::Int) = [ randomchromosome(l, k) for _ in 1 : n ]


function select(population::Vector{Chromosome}, fitnesses::Vector{Vector{Float64}}, params::Params)
    n = params.popsize
    m = params.popsize - params.nelites

    # total     = sum(fitnesses)
    # mean      = total / length(population)
    # sd        = stdm(fitnesses, mean)

    # sigma(fit) = sd == 0 ? 1.0 : (1 + (fit - mean) / 2sd)

    # sigmas     = map(sigma, fitnesses)
    # sigmas     = map(e -> e < 0 ? 0.1 : 0, sigmas)
    # sigmatotal = sum(sigmas)

    # weights = map(s -> s / sigmatotal, sigmas)

    # sample(population, Weights(weights), n)

    ranking = sort(1 : n, by=i -> fitnesses[i])
    rank(i) = findfirst(r -> r == i, ranking)

    amax = 1.2
    amin = 0.8

    p(i) = (1 / n) * (amin + (amax - amin) * ((rank(i) - 1) / (n - 1)))

    weights = map(i -> p(i), 1 : n)

    # boltzs = map(w -> exp(w / (gen / lastimprov)), weights)
    # boltzm = sum(boltzs) / length(boltzs)

    # weights = map(b -> b / boltzm, boltzs)

    sample(population, Weights(weights), m, replace=false)
end


function selectelites(population::Vector{Chromosome}, fitnesses::Vector{Vector{Float64}}, nelites::Int)
    elites = sort(1:length(population), by=i -> fitnesses[i], rev=true)[1:nelites]
    population[elites]
end


function reproduce(selection::Vector{Chromosome}, params::Params)
    nchildren = params.popsize - params.nelites

    collect(Iterators.flatten([
        begin
            parents = sample(selection, 2)

            offspring = crossover(parents..., params.pcross)

            foreach(child -> mutate!(child, params.pmut), offspring)

            offspring
        end
    for _ in 1 : ceil(Int, nchildren / 2)]))
end


function crossover(mother::Chromosome, father::Chromosome, pcross::Float64)
    if rand() < pcross
        l = length(mother)

        point1 = rand(1 : l - 1)
        point2 = rand(point1 + 1: l)

        left  = [ mother[1 : point1]; father[point1 + 1 : point2]; mother[point2 + 1 : l] ]
        right = [ father[1 : point1]; mother[point1 + 1 : point2]; father[point2 + 1 : l] ]
        
        [left, right]
    else
        [deepcopy(mother), deepcopy(father)]
    end
end


function mutate!(chrom::Chromosome, pmut::Float64)
    l = length(chrom)

    if rand() < pmut
        for (i, gene) in enumerate(chrom)
            if rand() < 1 / l
                chrom[i] = !gene
            end
        end
    end
end


function solve(instance::MRMPInstance, chrom::Chromosome, zerosol::Solution)
    if sum(chrom) == 0
        return zerosol
    end

    sol  = deepcopy(zerosol)
    inst = deepcopy(instance)

    inst.robots = map(last, filter(first, collect(zip(chrom, inst.robots))))

    # inst.obstacles = DynamicObstacles([
    #     map(last, filter(p -> !first(p), collect(zip(chrom, config))))
    #     for config in sol
    # ])

    chromsol = cbs(inst)[1]

    if !isnothing(chromsol)
        mergesolutions(sol, chromsol, chrom)
    else
        nothing
    end
end


function mergesolutions(left::Solution, right::Solution, chrom::Chromosome)
    if length(left) < length(right)
        for _ in length(left) + 1 : length(right)
            push!(left, left[end])
        end
    end

    if length(right) < length(left)
        for _ in length(right) + 1 : length(left)
            push!(right, right[end])
        end
    end

    for (time, config) in enumerate(left)
        for (i, r) in enumerate(findall(chrom))
            config[r] = right[time][i]
        end
    end

    left
end


function randomchromosome(l::Int, k::Int)
    chrom = falses(l)

    for gene in 1 : l
        chrom[gene] = rand() < (k / l)
    end

    chrom
end


end