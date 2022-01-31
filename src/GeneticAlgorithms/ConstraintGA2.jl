module ConstraintGA2

using MotionPlanning.Collisions
using MotionPlanning.Constraints
using MotionPlanning.Model
using MotionPlanning.MultiRobotPlanning.CBS
using MotionPlanning.MultiRobotPlanning.Metrics
using MotionPlanning.MultiRobotPlanning.Shadoks
using MotionPlanning.SingleRobotPlanning
using MotionPlanning.Utils: @timeout

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


const Chromosome = ConstraintMatrix


const ChromDims = NTuple{4, Int}


const Genes = SparseArray{Int16, 3}


zerochrom(n, w, h, d) = SparseArray{Int16}(zeros(n, w, h, d))


genes(r::Int, chrom::Chromosome) = chrom[r, :, :, :]


genes(r::Int, pos::Pos, chrom::Chromosome) = chrom[r, pos[1], pos[2], :]


genes(r::Int, pos::Pos, t::Int, chrom::Chromosome) = chrom[r, pos[1], pos[2], t]


const constraintvals = [ 2, 3, 5, 7, 11, 13 ]


function evolve(inst::MRMPInstance, params::Params; maxgens=Inf)
    n    = length(inst.robots)
    w, h = inst.dims
    d    = 2 * (w + h)

    chromdims = (n, w, h, d)

    paths     = Dict{UInt64, Union{Path, Nothing}}()
    fitness   = Dict{UInt64, Vector{Float64}}()

    initplan  = [ astar(r.pos, r.target, inst) for r in inst.robots ]
    initsol   = plantosolution(initplan)
    initchrom = zerochrom(chromdims...)

    weights = fill(log(prod(chromdims) * 7) / (prod(chromdims) * 7), chromdims)

    ncolls = length(findcollisions(initsol, inst))

    if ncolls == 0
        return initsol
    end

    for (r, path) in enumerate(initplan)
        paths[hash((r, genes(r, initchrom)))] = path
    end

    fitness[hash(initchrom)] = f(initchrom, initsol, inst)

    population = populate(params.popsize, weights, chromdims)

    kappa      = 1
    tau        = 1
    generation = 1
    lastimprov = 1
    mincolls   = ncolls
    diversity  = length(unique(population)) / length(population)

    try
        while generation < maxgens
            # println("generation: $generation, kappa: $k, best: $mincolls, constraints: $constraints, diversity: $diversity")
            println("generation: $generation, kappa: $kappa, tau: $tau, best: $mincolls, diversity: $diversity")

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
                    fitness[h] = f(chrom, sol, inst)

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
            diversity   = length(unique(population)) / length(population)
            kappa       = maximum([ sum(sum(chrom[r, :, :, :]) > 0 for r in 1 : n) for chrom in population ])
            tau         = maximum([ sum([ nconstraints(g) for g in chrom ]) for chrom in population ])
        end
    catch _
        println("interrupted")

        order = sort(population, by=chrom -> fitness[hash(chrom)], rev=true)

        (solve!(paths, order[1], inst), order[1])
    end

    order = sort(population, by=chrom -> fitness[hash(chrom)], rev=true)

    (solve!(paths, order[1], inst), order[1])
end


function f(chrom::Chromosome, sol::Union{Solution, Nothing}, inst::MRMPInstance)
    if isnothing(sol)
        [ 0.0, 0.0, 0.0 ]
    else
        colls   = length(findcollisions(sol, inst))
        sumdist = totaldist(sol, :solution)
        maxdist = makespan(sol, :solution)
        tau     = sum([ nconstraints(g) for g in chrom ])

        sig(x) = exp(-x) / (1 + exp(-x))

        [ sig(colls), sig(tau), sig(sumdist), sig(maxdist) ]
    end
end


function populate(popsize::Int, weights, chromdims)
    n, w, h, d = chromdims
    [ begin 
        chrom = zerochrom(n, w, h, d)

        for r in 1:n, x in 1:w, y in 1:h, t in 1:d
            if rand() < weights[r, x, y, t]
                g = 0

                for c in constraintvals
                    g = rand() < (1 / 5) ? xmult(g, c) : g
                end

                chrom[r, x, y, t] = g
            end
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

    boltzs = map(w -> exp(w / (gen / min(lastimprov^2, gen))), weights)
    boltzm = sum(boltzs) / length(boltzs)

    weights = map(b -> b / boltzm, boltzs)

    sample(population, Weights(weights), m, replace=false)
end


function selectelites(population::Vector{Chromosome}, fitnesses::Vector{Vector{Float64}}, nelites::Int)
    elites = sort(1:length(population), by=i -> fitnesses[i], rev=true)[1:nelites]
    population[elites]
end


function reproduce(selection::Vector{Chromosome}, params::Params, paths::Dict{UInt64, Union{Path, Nothing}}, inst::MRMPInstance, lastimprov, gen)
    n, w, h, d = size(selection[1])

    nchildren = params.popsize - params.nelites

    collect(Iterators.flatten([
        begin
            parents   = sample(selection, 2)

            mutmut = 0.5 + exp(min(lastimprov^2, gen) / gen) / (1 + exp(lastimprov / gen))

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
    paths  :: Dict{UInt64, Union{Path, Nothing}},
    inst   :: MRMPInstance,
    pcross :: Float64
)
    chromdims = size(mother)

    if rand() < pcross
        n, w, h, d = chromdims

        msol = solve!(paths, mother, inst)
        mcolls = isnothing(msol) ? Set{Collision}() : findcollisions(msol, inst)

        fsol = solve!(paths, father, inst)
        fcolls = isnothing(fsol) ? Set{Collision}() : findcollisions(fsol, inst)

        colls = union(mcolls, fcolls)

        weights = length(colls) > 0 ? weightsfromcollisions(colls, chromdims) : zeros(chromdims...)

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

        # for r in 1 : r1, x in 1 : x1, y in 1 : y1, t in 1 : t1
        #     left[r, x, y, t]  = mother[r, x, y, t]
        #     right[r, x, y, t] = father[r, x, y, t]
        # end

        # for r in r1 : r2, x in x1 : x2, y in y1 : y2, t in t1 : t2
        #     left[r, x, y, t]  = father[r, x, y, t]
        #     right[r, x, y, t] = mother[r, x, y, t]
        # end

        # for r in r2 + 1 : n, x in x2 + 1 : w, y in y2 : h, t in t2 + 1 : d
        #     left[r, x, y, t]  = mother[r, x, y, t]
        #     right[r, x, y, t] = father[r, x, y, t]
        # end

        for r in 1 : r1, x in 1 : x1, y in 1 : y1, t in 1 : t1
            left[r, x, y, t]  = passgene(mother[r, x, y, t], weights[r, x, y, t])
            right[r, x, y, t] = passgene(father[r, x, y, t], weights[r, x, y, t])
        end

        for r in r1 : r2, x in x1 : x2, y in y1 : y2, t in t1 : t2
            left[r, x, y, t]  = passgene(father[r, x, y, t], weights[r, x, y, t])
            right[r, x, y, t] = passgene(mother[r, x, y, t], weights[r, x, y, t])
        end

        for r in r2 + 1 : n, x in x2 + 1 : w, y in y2 : h, t in t2 + 1 : d
            left[r, x, y, t]  = passgene(mother[r, x, y, t], weights[r, x, y, t])
            right[r, x, y, t] = passgene(father[r, x, y, t], weights[r, x, y, t])
        end

        [ left, right ]
    else
        [ deepcopy(mother), deepcopy(father) ]
    end
end


function mutate!(chrom::Chromosome, pmut::Float64)
    n, w, h, d = size(chrom)

    l = prod(size(chrom))

    if rand() < pmut
        for r in 1:n, x in 1:w, y in 1:h, t in 1:d
            if rand() < log(n) / l
                for c in constraintvals
                    if rand() < (1 / 6)
                        chrom[r, x, y, t] = flip(chrom[r, x, y, t], c)
                    end
                end
            end
        end
    end
end


function solve!(paths::Dict{UInt64, Union{Path, Nothing}}, chrom::Chromosome, inst::MRMPInstance)
    updatepaths!(paths, chrom, inst)

    plan = [ paths[hash((r, genes(r, chrom)))] for r in 1:size(chrom)[1] ]

    if any(map(isnothing, plan))
        return nothing
    end

    plantosolution(plan)
end


function updatepaths!(paths::Dict{UInt64, Union{Path, Nothing}}, chrom::Chromosome, inst::MRMPInstance)
    chromdims = size(chrom)

    for r in 1 : chromdims[1]
        g = genes(r, chrom)
        h = hash((r, g))

        if !haskey(paths, h)
            robot    = inst.robots[r]
            paths[h] = astar(robot.pos, robot.target, inst; constmat=chrom, r=r)
        end
    end
end


# function genestoconstraints(r::Int, g::Genes)
#     w, h, d = size(g)

#     Vector{Constraint}(filter(!isnothing, [
#         g[x, y, t] ? ClashConstraint(t, (x, y), r) : nothing
#         for x in 1:w, y in 1:h, t in 1:d
#     ]))
# end


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

    rdist = map(r -> (total - rcolls[r]) / total, 1:n)

    for x in 1:w, y in 1:h, t in 1:d
        colldist[(x, y, t)] = (maxdist - collisiondist((x, y, t), collvecs)) / maxdist
    end

    # distrib = Biweight(1, 0.25)

    Float64[
        # 2 * log(n) * (1 + rand(dist)) * ((r, (x, y), t) ∈ colls ? ℯ : 1) / total
        rdist[r] * colldist[x, y, t] / n
        for r in 1:n, x in 1:w, y in 1:h, t in 1:d
    ]
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


function xmult(a, b)
    if a > 0 && a % b == 0
        a
    else
        max(1, a) * b
    end
end


function xdiv(a, b)
    if a > 0 && a % b == 0
        a // b
    else
        a
    end
end


function flip(a, b)
    if a > 0 && a % b == 0
        a // b
    else
        max(1, a) * b  
    end
end


function passgene(gene::Int16, weight::Float64)
    if rand() < weight
        for c in constraintvals
            if rand() < (1 / 6)
                gene = xmult(gene, c)
            end
        end
    end

    gene
end


function nconstraints(gene::Int16)
    n = 0

    if gene > 0
        for c in constraintvals
            if gene % c == 0
                n += 1
            end
        end
    end

    n
end


end