module CouplingGA3

using MotionPlanning.Collisions
using MotionPlanning.Model
using MotionPlanning.MultiRobotPlanning.CBS
using MotionPlanning.MultiRobotPlanning.Metrics
using MotionPlanning.MultiRobotPlanning.Shadoks
using MotionPlanning.SingleRobotPlanning

using Graphs
using StatsBase
using Statistics

export
    evolve,
    iterevolve,
    Params


const Chromosome = SimpleGraph{Int64}


struct Params
    popsize  :: Int
    nelites  :: Int
    pcross   :: Float64
    pmut     :: Float64
    kmax     :: Int
end


# function iterevolve(inst::MRMPInstance, params::Params, initialsol::Union{Solution, Nothing}=nothing, collthresh::Int=0)
#     try
#         sol = evolve(inst::MRMPInstance, params, initialsol)    

#         ncolls = length(findcollisions(sol, inst))

#         if ncolls <= collthresh
#             return sol
#         end

#         iterevolve(inst, params, sol, collthresh)
#     catch _
#         initialsol
#     end
# end


function evolve(inst::MRMPInstance, params::Params, initialsol::Union{Solution, Nothing}=nothing; kinit=1)
    chromsize = length(inst.robots)

    solutions      = Dict{UInt64, Union{Solution, Nothing}}()
    localsolutions = Dict{Set{Int}, Union{Solution, Nothing}}()
    fitness        = Dict{UInt64, Vector{Float64}}()

    if isnothing(initialsol)
        zerosol   = plantosolution([ astar(r.pos, r.target, inst) for r in inst.robots ])
    else
        zerosol = initialsol
    end

    ncolls = length(findcollisions(zerosol, inst))

    population = populate(params.popsize, chromsize, kinit)

    kappa      = maximum(κ(chrom) for chrom in population)
    lambda     = maximum(λ(chrom) for chrom in population)
    generation = 1
    best       = ncolls
    lastimprov = 1

    # try
        while best > 0 # kappa <= params.kmax && generation < 1025
            println("generation: $generation, kappa: $kappa, lambda: $lambda, best: $best")

            # generate the next generation
            if generation > 1
                for chrom in population
                    if !haskey(fitness, hash(chrom))
                        fitness[hash(chrom)] = f(chrom, solutions[hash(chrom)], inst)
                    end
                end

                fitnesses = map(chrom -> fitness[hash(chrom)], population)

                selection = select(population, fitnesses, params, generation, lastimprov)

                elites = selectelites(population, fitnesses, params.nelites)

                selection = [ elites; selection ]

                children = reproduce(selection, params)

                population = [ elites; children ]
            end

            # solve instances and find fitnesses for newly discovered chromosomes
            for chrom in population
                solution = if !haskey(solutions, hash(chrom))
                    solve!(solutions, localsolutions, chrom, inst, zerosol)
                else
                    solutions[hash(chrom)]
                end
                
                if !haskey(fitness, hash(chrom))
                    fitness[hash(chrom)] = f(chrom, solution, inst)

                    if !isnothing(solution)
                        ncolls = length(findcollisions(solution, inst))

                        if ncolls < best
                            best = ncolls
                            lastimprov = 1
                        end
                    end
                end
            end

            if lastimprov > 1
                lastimprov += 1
            end

            kappa  = maximum(κ(chrom) for chrom in population)
            lambda = maximum(λ(chrom) for chrom in population)

            generation += 1
        end
    # catch _
    #     println("interrupted")

    #     best = first(sort(population, by=chrom -> fitness[chrom], rev=true))

    #     solution[best]
    # end

    best = first(sort(population, by=chrom -> fitness[chrom], rev=true))

    solutions[hash(best)]
end


function f(chrom::Chromosome, sol::Union{Solution, Nothing}, inst::MRMPInstance)
    if isnothing(sol)
        [0.0, 0.0, 0.0, 0.0]
    else
        ncolls = length(findcollisions(sol, inst))
        sum    = totaldist(sol, :solution)
        max    = makespan(sol, :solution)
        kappa  = κ(chrom)

        sig(x) = exp(-x) / (1 + exp(-x))

        [ sig(ncolls), sig(kappa), sig(sum), sig(max) ]
    end
end


populate(p::Int, n::Int, k::Int) = [ SimpleGraph(n, 1) for _ in 1 : p ]


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
        n = nv(mother)

        r1, s1 = rand(1:n - 1), rand(1:n - 1)
        r2, s2 = rand(r1 + 1:n), rand(s1 + 1:n)

        leftedges  = []
        rightedges = []
        for r in 1:r1, s in 1:s2
            if has_edge(mother, r, s)
                push!(leftedges, Edge(r, s))
            end

            if has_edge(father, r, s)
                push!(rightedges, Edge(r, s))
            end
        end

        for r in r1+1:r2, s in s1+1:s2
            if has_edge(father, r, s)
                push!(leftedges, Edge(r, s))
            end

            if has_edge(mother, r, s)
                push!(rightedges, Edge(r, s))
            end
        end

        for r in r2:n, s in s2:n
            if has_edge(mother, r, s)
                push!(leftedges, Edge(r, s))
            end

            if has_edge(father, r, s)
                push!(rightedges, Edge(r, s))
            end
        end

        left = SimpleGraph(n)

        for edge in leftedges
            add_edge!(left, edge)
        end

        right = SimpleGraph(n)

        for edge in rightedges
            add_edge!(right, edge)
        end
            
        [left, right]
    else
        [deepcopy(mother), deepcopy(father)]
    end
end


function mutate!(chrom::Chromosome, pmut::Float64)
    n = size(chrom)[1]

    if rand() < pmut
        for r in 1:n, s in 1:n
            if rand() < 1 / (n^2 - n)
                if has_edge(chrom, r, s)
                    rem_edge!(chrom, r, s)
                else
                    add_edge!(chrom, r, s)
                end
            end
        end
    end
end


function solve!(
    solutions      :: Dict{UInt64, Union{Solution, Nothing}}, 
    localsolutions :: Dict{Set{Int}, Union{Solution, Nothing}},
    chrom          :: Chromosome, 
    instance       :: MRMPInstance, 
    zerosol        :: Solution
)
    if ne(chrom) == 0
        return zerosol
    end

    groups = connected_components(chrom)

    sol = deepcopy(zerosol)

    for group in groups
        if haskey(localsolutions, Set(group))
            groupsol = localsolutions[Set(group)]

            if isnothing(groupsol)
                return nothing
            else
                sol = mergesolutions(sol, localsolutions[Set(group)], group)
            end
        else
            inst = deepcopy(instance)

            inst.robots = filter(r -> r.id ∈ group, inst.robots)

            result = cbs(inst)

            groupsol = isnothing(result) ? nothing : result[1]

            localsolutions[Set(group)] = groupsol

            if isnothing(groupsol)
                return nothing
            else
                sol = mergesolutions(sol, localsolutions[Set(group)], group)
            end
        end
    end

    solutions[hash(chrom)] = sol

    sol
end


function mergesolutions(left::Solution, right::Solution, group::Vector{Int})
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
        for (i, r) in enumerate(group)
            config[r] = right[time][i]
        end
    end

    left
end


κ(chrom::Chromosome) = maximum(length(group) for group in connected_components(chrom))


λ(chrom::Chromosome) = ne(chrom)


end