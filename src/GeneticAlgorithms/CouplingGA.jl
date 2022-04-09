module CouplingGA

using MotionPlanning.Collisions
using MotionPlanning.Model
using MotionPlanning.MultiRobotPlanning.CBS
using MotionPlanning.MultiRobotPlanning.Metrics
using MotionPlanning.MultiRobotPlanning.PriorityPlanning
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


function evolve(inst::MRMPInstance, params::Params, initialsol::Union{Solution, Nothing}=nothing; kinit=1, maxgens=1024)
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
    diversity  = length(unique(population)) / length(population)

    try
        while best > 0 && generation < maxgens && kappa <= params.kmax
            println("generation: $generation, kappa: $kappa, lambda: $lambda, diversity: $diversity, best: $best")

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

            kappa     = maximum(κ(chrom) for chrom in population)
            lambda    = maximum(λ(chrom) for chrom in population)
            diversity = length(unique(population)) / length(population)

            generation += 1
        end
    catch _
        println("interrupted")

        best = first(sort(population, by=chrom -> fitness[hash(chrom)], rev=true))

        solution[hash(best)]
    end

    best = first(sort(population, by=chrom -> fitness[hash(chrom)], rev=true))

    solutions[hash(best)]
end


sig(x) = 1 / (1 + exp(-x))


relent(c, n, maxdist, q) = let
	p = c / (n * maxdist)
	p * log2(p / q) + (1 - p) * log2((1 - p) / (1 - q))
end


function f(chrom::Chromosome, sol::Union{Solution, Nothing}, inst::MRMPInstance)
    if isnothing(sol)
        [0.0]
    else
        n       = size(chrom)[1]
        colls   = length(findcollisions(sol, inst))
        maxdist = makespan(sol, :solution)
        kappa  = κ(chrom)

        divergence = colls == 0 ? log2(5) : relent(colls, n, maxdist, 0.8)

        kpenalty = sig((nv(chrom) - kappa) / nv(chrom))

        [ divergence  * kpenalty ]
    end
end


populate(p::Int, n::Int, k::Int) = [ SimpleGraph(n, 1) for _ in 1 : p ]


function select(population::Vector{Chromosome}, fitnesses::Vector{Vector{Float64}}, params::Params, gen, lastimprov)
    n = params.popsize
    m = params.popsize - params.nelites

    fitnesses = map(first, fitnesses)
    
    totalfitness = sum(fitnesses)

    pointerdist = totalfitness / m

    start = rand() * pointerdist

    pointers = [ start + i * pointerdist for i in 1 : m -1 ]

    roulettewheelselection(population, fitnesses, pointers)
end


function roulettewheelselection(population, fitnesses, pointers)
    selected = Chromosome[]

    for p in pointers
        i = 0

        while sum(fitnesses[1 : i]) < p
            i += 1
        end

        push!(selected, population[i])
    end

    selected
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

            foreach(child -> 
                while κ(child) > params.kmax
                    edge = sample(collect(edges(child)))
                    rem_edge!(child, edge)
                end, offspring
            )

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