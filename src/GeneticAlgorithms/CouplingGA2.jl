module CouplingGA2

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


const Chromosome = Tuple{UInt64, BitVector}


struct Params
    popsize  :: Int
    nelites  :: Int
    pcross   :: Float64
    pmut     :: Float64
    kmax     :: Int
end


function evolve(inst::MRMPInstance, params::Params, initialsol::Union{Solution, Nothing}=nothing; kinit=1)
    chromsize = length(inst.robots)

    solutions = Dict{UInt64, Union{Solution, Nothing}}()
    chromsols = Dict{Chromosome, UInt64}()
    fitness   = Dict{Chromosome, Vector{Float64}}()

    if isnothing(initialsol)
        # zerosol   = shadoks(inst)
        zerosol = plantosolution([ ep(r.pos, r.target, inst) for r in inst.robots ])
    else
        zerosol = initialsol
    end

    ncolls = length(findcollisions(zerosol, inst))

    population = populate(zerosol, params.popsize, chromsize, kinit)

    solutions[hash(zerosol)] = zerosol

    k          = kinit
    generation = 1
    best       = ncolls
    diversity  = length(unique(population)) / length(population)
    lastimprov = 1

    # try
        while k <= params.kmax && generation < 2048
            println("generation: $generation, kappa: $k, best: $best, diversity: $diversity")

            # generate the next generation
            if generation > 1
                fitnesses = map(chrom -> fitness[chrom], population)

                selection = select(population, fitnesses, params, generation, lastimprov)

                elites = selectelites(population, fitnesses, params.nelites)

                selection = [ elites; selection ]

                children = reproduce(selection, chromsols, solutions, params)

                population = [ elites; children ]
            end

            # solve instances and find fitnesses for newly discovered chromosomes
            for chrom in population
                if !haskey(chromsols, chrom)
                    init     = solutions[chrom[1]]
                    solution = solve(inst, chrom, init)

                    if isnothing(solution)
                        println("nothing")
                        chromsols[chrom] = hash(nothing)
                        fitness[chrom] = [0, 0, 0]
                    else
                        ncolls = length(findcollisions(solution, inst))

                        best = min(best, ncolls)
                        if ncolls < best
                            best = ncolls
                            lastimprov = 1
                        end

                        if !haskey(solutions, hash(solution))
                            solutions[hash(solution)] = solution
                        end

                        chromsols[chrom] = hash(solution)
                        fitness[chrom]   = f(solution, inst)
                    end
                end
            end

            if lastimprov > 1
                lastimprov += 1
            end

            k = maximum(sum(chrom[2]) for chrom in population)
            diversity = length(unique(population)) / length(population)

            generation += 1
        end
    # catch _
    #     println("interrupted")

    #     best = first(sort(population, by=chrom -> fitness[chrom], rev=true))

    #     return solution[best[1]]
    # end

    best = first(sort(population, by=chrom -> fitness[chrom], rev=true))

    solutions[best[1]]
end


function f(sol::Solution, inst::MRMPInstance)
    ncolls = length(findcollisions(sol, inst))
    sum    = totaldist(sol, :solution)
    max    = makespan(sol, :solution)

    sig(x) = exp(-x) / (1 + exp(-x))

    [ sig(ncolls), sig(sum), sig(max) ]
end


populate(initialsol::Solution, n::Int, l::Int, k::Int) = [ (hash(initialsol), randomchromosome(l, k)) for _ in 1 : n ]


function select(population::Vector{Chromosome}, fitnesses::Vector{Vector{Float64}}, params::Params, gen, lastimprov)
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

    boltzs = map(w -> exp(w / (gen / lastimprov)), weights)
    boltzm = sum(boltzs) / length(boltzs)

    weights = map(b -> b / boltzm, boltzs)

    sample(population, Weights(weights), m, replace=false)
end


function selectelites(population::Vector{Chromosome}, fitnesses::Vector{Vector{Float64}}, nelites::Int)
    elites = sort(1:length(population), by=i -> fitnesses[i], rev=true)[1:nelites]
    population[elites]
end


function reproduce(selection::Vector{Chromosome}, chromsols::Dict{Chromosome, UInt64}, solutions::Dict{UInt64, Union{Nothing, Solution}}, params::Params)
    nchildren = params.popsize - params.nelites

    collect(Iterators.flatten([
        begin
            parents = sample(selection, 2)

            offspring = crossover(parents..., chromsols, solutions, params.pcross)

            foreach(child -> mutate!(child, solutions, params.pmut), offspring)

            offspring
        end
    for _ in 1 : ceil(Int, nchildren / 2)]))
end


function crossover(mother::Chromosome, father::Chromosome, chromsols::Dict{Chromosome, UInt64}, solutions::Dict{UInt64, Union{Nothing, Solution}}, pcross::Float64)
    if rand() < pcross
        l = length(mother)

        point1 = rand(1 : l - 1)
        point2 = rand(point1 + 1: l)

        mothersol = chromsols[mother]
        fathersol = chromsols[father]

        left  = (mothersol, [ mother[2][1 : point1]; father[2][point1 + 1 : point2]; mother[2][point2 + 1 : l] ])
        right = (fathersol, [ father[2][1 : point1]; mother[2][point1 + 1 : point2]; father[2][point2 + 1 : l] ])
        
        [left, right]
    else
        [deepcopy(mother), deepcopy(father)]
    end
end


function mutate!(chrom::Chromosome, solutions::Dict{UInt64, Union{Nothing, Solution}}, pmut::Float64)
    l = length(chrom)

    if rand() < pmut
        for (i, gene) in enumerate(chrom[2])
            if rand() < 1 / l
                chrom[2][i] = !gene
            end
        end

        if rand() < 1 / l
            chrom = (rand(collect(keys(solutions))), chrom[2])
        end
    end
end


function solve(instance::MRMPInstance, chrom::Chromosome, initsol::Solution)
    if sum(chrom[2]) == 0
        return initsol
    end

    sol  = deepcopy(initsol)
    inst = deepcopy(instance)

    inst.robots = map(last, filter(first, collect(zip(chrom[2], inst.robots))))

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
        for (i, r) in enumerate(findall(chrom[2]))
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