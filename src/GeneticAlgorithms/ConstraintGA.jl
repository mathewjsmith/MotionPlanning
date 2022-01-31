module ConstraintGA

using MotionPlanning.Collisions
using MotionPlanning.Constraints
using MotionPlanning.Model
using MotionPlanning.MultiRobotPlanning.CBS
using MotionPlanning.MultiRobotPlanning.Metrics
using MotionPlanning.MultiRobotPlanning.Shadoks
using MotionPlanning.SingleRobotPlanning

using Distributions
using SparseArrayKit
using StatsBase
using Statistics

export
    evolve,
    Params


struct Params
    popsize  :: Int
    nelites  :: Int
    pcross   :: Float64
    pmut     :: Float64
    kmax     :: Int
end


const Chromosome = SparseArray{Bool, 4}


const ChromDims = NTuple{4, Int}


const Genes = SparseArray{Bool, 3}


zerochrom(n, w, h, d) = SparseArray{Bool}(falses(n, w, h, d))


genes(r::Int, chrom::Chromosome) = chrom[r, :, :, :]


genes(r::Int, pos::Pos, chrom::Chromosome) = chrom[r, pos[1], pos[2], :]


genes(r::Int, pos::Pos, t::Int, chrom::Chromosome) = chrom[r, pos[1], pos[2], t]


function evolve(inst::MRMPInstance, params::Params)
    n    = length(inst.robots)
    w, h = inst.dims
    d    = 2 * (w + h)
    chromdims = (n, w, h, d)

    paths     = Dict{UInt64, Path}()
    fitness   = Dict{UInt64, Vector{Float64}}()

    initplan  = [ astar(r.pos, r.target, inst) for r in inst.robots ]
    initsol   = plantosolution(initplan)
    initchrom = zerochrom(chromdims...)

    colls = findcollisions(initsol, inst)

    weights = weightsfromcollisions(colls, chromdims)

    ncolls = length(findcollisions(initsol, inst))

    if ncolls == 0
        return initsol
    end

    for (r, path) in enumerate(initplan)
        paths[hash((r, genes(r, initchrom)))] = path
    end

    fitness[hash(initchrom)] = f(initsol, inst)

    population = populate(params.popsize, weights, chromdims)

    k           = 1
    generation  = 1
    mincolls    = ncolls
    lastimprov  = 1
    constraints = maximum(sum(chrom) for chrom in population)
    diversity   = length(unique(population)) / length(population)

    try
        while k <= params.kmax # && generation < 256
            println("generation: $generation, kappa: $k, best: $mincolls, constraints: $constraints, diversity: $diversity")

            # generate the next generation
            if generation > 1
                fitnesses = map(chrom -> fitness[hash(chrom)], population)

                selection = select(population, fitnesses, params, generation, lastimprov)

                elites = selectelites(population, fitnesses, params.nelites)

                selection = [ elites; selection ]

                children = reproduce(selection, params, paths, inst, lastimprov, generation)

                population = [ elites; children ]
            end

            # solve instances and find fitnesses for newly discovered chromosomes
            for chrom in population
                h = hash(chrom)
                
                if !haskey(fitness, h)
                    sol        = solve!(paths, chrom, inst)
                    fitness[h] = f(sol, inst)

                    ncolls   = isnothing(sol) ? Inf : length(findcollisions(sol, inst))

                    if ncolls < mincolls
                        mincolls = ncolls
                        lastimprov = 1
                    end
                end
            end

            if lastimprov > 1
                lastimprov += 1
            end

            generation += 1
            constraints = maximum(sum(chrom) for chrom in population)
            diversity   = length(unique(population)) / length(population)
            k           = maximum(sum(any(chrom[r, :, :, :]) for r in 1:n) for chrom in population)
        end
    catch _
        println("interrupted")

        order = sort(population, by=chrom -> fitness[hash(chrom)], rev=true)

        (solve!(paths, order[1], inst), order[1])
    end

    order = sort(population, by=chrom -> fitness[hash(chrom)], rev=true)

    (solve!(paths, order[1], inst), order[1])
end


function f(sol::Solution, inst::MRMPInstance)
    col = length(findcollisions(sol, inst))
    sum = totaldist(sol, :solution)
    max = makespan(sol, :solution)

    sig(x) = exp(-x) / (1 + exp(-x))

    [ sig(col), sig(sum), sig(max) ]
end


function populate(popsize::Int, weights, chromdims)
    n, w, h, d = chromdims
    [ begin 
        chrom = zerochrom(n, w, h, d)

        for r in 1:n, x in 1:w, y in 1:h, t in 1:d
            chrom[r, x, y, t] = rand() < weights[r, x, y, t] 
        end

        chrom
    end
    for _ in 1:popsize ]
end


function select(population::Vector{Chromosome}, fitnesses::Vector{Vector{Float64}}, params::Params, gen, lastimprov)
    n = params.popsize
    m = params.popsize - params.nelites

    # total     = sum(fitnesses)
    # mean      = total / n
    # sd        = stdm(fitnesses, mean)

    # sigma(fit) = sd == 0 ? 1.0 : (1 + (fit - mean) / 2sd)

    # sigmas     = map(sigma, fitnesses)
    # sigmas     = map(e -> e < 0 ? 0.1 : 0, sigmas)
    # sigmatotal = sum(sigmas)

    # weights = map(s -> s / sigmatotal, sigmas)

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


function reproduce(selection::Vector{Chromosome}, params::Params, paths::Dict{UInt64, Path}, inst::MRMPInstance, lastimprov, gen)
    n, w, h, d = size(selection[1])

    nchildren = params.popsize - params.nelites

    collect(Iterators.flatten([
        begin
            parents   = sample(selection, 2)

            mutmut = 0.5 + exp(lastimprov / gen) / (1 + exp(lastimprov / gen))

            offspring = crossover(parents..., paths, inst, params.pcross)

            foreach(child -> mutate!(child, params.pmut * mutmut), offspring)

            offspring
        end
    for _ in 1 : ceil(Int, nchildren / 2)
    ]))
end


function crossover(
    mother :: Chromosome, 
    father :: Chromosome, 
    paths  :: Dict{UInt64, Path},
    inst   :: MRMPInstance,
    pcross :: Float64
)
    chromdims = size(mother)

    if rand() < pcross
        n, w, h, d = chromdims

        colls = union(findcollisions(solve!(paths, mother, inst), inst), findcollisions(solve!(paths, father, inst), inst))

        weights = weightsfromcollisions(colls, chromdims)

        r1, x1, y1, t1 = (
            rand(1 : n - 1),
            rand(1 : w - 1),
            rand(1 : h - 1),
            rand(1 : d - 1)
        )

        r2, x2, y2, t2 = (
            rand(r1 + 1 : n),
            rand(x1 + 1 : w),
            rand(y1 + 1 : h),
            rand(t1 + 1 : d)
        )

        left  = zerochrom(chromdims...)
        right = zerochrom(chromdims...)

        for r in 1 : r1, x in 1 : x1, y in 1 : y1, t in 1 : t1
            left[r, x, y, t]  = mother[r, x, y, t] || rand() < weights[r, x, y, t]
            right[r, x, y, t] = father[r, x, y, t] || rand() < weights[r, x, y, t]
        end

        for r in r1 : r2, x in x1 : x2, y in y1 : y2, t in t1 : t2
            left[r, x, y, t]  = father[r, x, y, t] || rand() < weights[r, x, y, t]
            right[r, x, y, t] = mother[r, x, y, t] || rand() < weights[r, x, y, t]
        end

        for r in r2 + 1 : n, x in x2 + 1 : w, y in y2 : h, t in t2 + 1 : d
            left[r, x, y, t]  = mother[r, x, y, t] || rand() < weights[r, x, y, t]
            right[r, x, y, t] = father[r, x, y, t] || rand() < weights[r, x, y, t]
        end

        [ left, right ]
    else
        [ deepcopy(mother), deepcopy(father) ]
    end
end


function mutate!(chrom::Chromosome, pmut::Float64)
    n, w, h, d = size(chrom)

    l = n * w * h * d

    if rand() < pmut
        for r in 1:n, x in 1:w, y in 1:h, t in 1:d
            if rand() < log(n) / l
                chrom[r, x, y, t] = !chrom[r, x, y, t]
            end
        end
    end
end


function solve!(paths::Dict{UInt64, Path}, chrom::Chromosome, inst::MRMPInstance)
    updatepaths!(paths, chrom, inst)

    plan = [ paths[hash((r, genes(r, chrom)))] for r in 1:size(chrom)[1] ]

    if any(map(isnothing, plan))
        return nothing
    end

    plantosolution(plan)
end


function updatepaths!(paths::Dict{UInt64, Path}, chrom::Chromosome, inst::MRMPInstance)
    for r in 1 : size(chrom)[1]
        g = genes(r, chrom)
        h = hash((r, g))

        if !haskey(paths, h)
            constraints = genestoconstraints(r, g)
            robot       = inst.robots[r]
            paths[h]    = astar(robot.pos, robot.target, inst, constraints)
        end
    end
end


function genestoconstraints(r::Int, g::Genes)
    w, h, d = size(g)

    Vector{Constraint}(filter(!isnothing, [
        g[x, y, t] ? ClashConstraint(t, (x, y), r) : nothing
        for x in 1:w, y in 1:h, t in 1:d
    ]))
end


function weightsfromcollisions(colls::Set{Collision}, chromdims::ChromDims)
    n, w, h, d = chromdims

    collvecs = collisionvectors(colls)

    maxdist = pointdist((1, 1, 1), (w, h, d))

    colldist = Dict{NTuple{3, Int}, Float64}()

    rcolls = Dict{Int, Int}([(r, 0) for r in 1:n ])

    for coll in collect(colls)
        robots = if isa(coll, Clash)
            collect(coll.robots)
        elseif isa(coll, Overlap)
            filter(!isnothing, [ coll.onrobot, coll.offrobot ])
        end

        for r in robots
            rcolls[r] += 1
        end
    end

    total = sum(values(rcolls))

    rdist = map(r -> rcolls[r] / total, 1:n)

    for x in 1:w, y in 1:h, t in 1:d
        colldist[(x, y, t)] = collisiondist((x, y, t), collvecs) / maxdist
    end

    distrib = Biweight(1, 0.25)

    Float64[
        # 2 * log(n) * (1 + rand(dist)) * ((r, (x, y), t) ∈ colls ? ℯ : 1) / total
        rand(distrib) * rdist[r] * colldist[x, y, t] / log(n / 2)
        for r in 1:n, x in 1:w, y in 1:h, t in 1:d
    ]
end


function uniformweights(chromdims::ChromDims)
    n, w, h, d = chromdims

    weightmat = Array{Float, 4}(chromdims)

    rand() < log(n) * (n * w * h * d)

end


function collisionvectors(colls::Set{Collision})
    collect(Iterators.flatten([
        if isa(coll, Clash)
            [ (coll.pos..., coll.time) for _ in collect(coll.robots) ]
        elseif isa(coll, Overlap)
            filter(!isnothing, [
                (coll.onsrc..., coll.time),
                (coll.ondst..., coll.time),
                !isnothing(coll.offdst) ? (coll.offdst..., coll.time) : nothing
            ])
        end
        for coll in collect(colls)
    ]))
end


collisiondist(v::NTuple{3, Int}, colls::Vector{NTuple{3, Int}}) = minimum(map(c -> pointdist(v, c), colls))


function pointdist(a, b)
    x1, y1, t1 = a
    x2, y2, t2 = b

    sqrt((x1 - x2)^2 + (y1 - y2)^2 + (t1 - t2)^2)
end


end