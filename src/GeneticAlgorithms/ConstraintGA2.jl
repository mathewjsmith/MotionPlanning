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
using Distributed
using SparseArrayKit
using StatsBase
using Statistics

export
    evolve,
    Params,
    nconstraints,
    GAStats


struct Params
    popsize  :: Int
    nelites  :: Int
    pcross   :: Float64
    pmut     :: Float64
end


struct GAStats
    bestfitness  :: Float64
    meanfitness  :: Float64
    mincolls     :: Int64
    nconstraints :: Float64
    diversity    :: Float64
end


const Chromosome = ConstraintMatrix


const ChromDims = NTuple{4, Int}


const Genes = SparseArray{Int16, 3}


zerochrom(n, w, h, d) = SparseArray{Int16}(zeros(n, w, h, d))


genes(r::Int, chrom::Chromosome) = chrom[r, :, :, :]


genes(r::Int, pos::Pos, chrom::Chromosome) = chrom[r, pos[1], pos[2], :]


genes(r::Int, pos::Pos, t::Int, chrom::Chromosome) = chrom[r, pos[1], pos[2], t]


const constraintvals = [ 2, 3, 5, 7, 11, 13 ]


function evolve(inst::MRMPInstance, params::Params; maxgens=Inf, kinit=nothing)
    n    = length(inst.robots)
    w, h = inst.dims
    d    = 2 * (w + h)

    chromdims = (n, w, h, d)

    paths     = Dict{UInt64, Union{Path, Nothing}}()
    fitness   = Dict{UInt64, Vector{Float64}}()

    initplan  = [ astar(r.pos, r.target, inst) for r in inst.robots ]
    initsol   = plantosolution(initplan)
    initchrom = zerochrom(chromdims...)

    colls = findcollisions(initsol, inst)

    ncolls = length(colls)

    if ncolls == 0
        return initsol
    end

    fitness[hash(initchrom)] = f(initchrom, initsol, inst)

    for (r, path) in enumerate(initplan)
        paths[hash((r, genes(r, initchrom)))] = path
    end

    consts = collect(Iterators.flatten(map(conflicttoconstraints, collect(colls))))

    population = let 
        k = isnothing(kinit) ? max(2, ceil(Int, log(ncolls))) : kinit
        pairs = mapslices(c -> [c], rand(collect(consts), (params.popsize, k)), dims=2)[:]
        map(c -> constraintstomatrix(Constraint[c...], chromdims), pairs)
    end

    # population = rand(initpool, params.popsize)

    # weights = fill(log(prod(chromdims)) / (prod(chromdims)), chromdims)
    # weights = fill(1 / (prod(chromdims)), chromdims)

    # population = populate(params.popsize, weights, chromdims)

    kappa      = 1
    tau        = 1
    generation = 1
    lastimprov = 1
    mincolls   = ncolls
    diversity  = length(unique(population)) / length(population)

    stats = GAStats[GAStats(0.0, 0.0, mincolls, 1, 1)]

    # try
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
            # results = @distributed (vcat) for chrom in population
            # results = pmap(chrom -> begin
#                h = hash(chrom)
#
#                (sol, newpaths) = solve(paths, chrom, inst)
#
#                fit = f(chrom, sol, inst)
#
#                ncolls = isnothing(sol) ? Inf : length(findcollisions(sol, inst))
#
#                (h, sol, fit, newpaths, ncolls)
#            end
            # end, population)

#            for (h, _, fit, newpaths, ncolls) in results
#                if !haskey(fitness, h)
#                    fitness[h] = fit
#                end 
#
#                for (gh, path) in newpaths
#                    paths[gh] = path
#                end
#
#                if ncolls < mincolls
#                    mincolls   = ncolls
#                    lastimprov = 1
#                end
#            end
                
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

            # for (h, sol, fit, ncolls) in results
            #     if sol != :seen
            #         fitness[h] = fit

            #         if ncolls < mincolls
            #             mincolls = ncolls
            #             lastimprov = 1
            #         end
            #     end
            # end

            if lastimprov > 1
                lastimprov += 1
            end

            generation += 1
            diversity   = length(unique(population)) / length(population)
            kappa       = maximum([ sum(sum(chrom[r, :, :, :]) > 0 for r in 1 : n) for chrom in population ])
            tau         = mean([ sum([ nconstraints(g) for g in chrom ]) for chrom in population ])

            fitnesses = map(chrom -> first(fitness[hash(chrom)]), population)

            push!(stats, GAStats(maximum(fitnesses), mean(fitnesses), mincolls, tau, diversity))

            if mincolls == 0
                break
            end
        end
    # catch _
    #     println("interrupted")

    #     order = sort(population, by=chrom -> fitness[hash(chrom)], rev=true)

    #     (solve(paths, order[1], inst)[1], order[1], stats)
    # end

    order = sort(population, by=chrom -> fitness[hash(chrom)], rev=true)

    (solve(paths, order[1], inst)[1], order[1], stats)
end


sig(x) = 1 / (1 + exp(-x))


relent(c, n, maxdist, q) = let
	p = c / (n * maxdist)
	p * log2(p / q) + (1 - p) * log2((1 - p) / (1 - q))
end


function f(chrom::Chromosome, sol::Union{Solution, Nothing}, inst::MRMPInstance)
    if isnothing(sol)
        [ 0.0, 0.0, 0.0 ]
    else
        colls     = length(findcollisions(sol, inst))

        if colls == 0
            [ 1.0 ]
        else
            n         = size(chrom)[1]
            maxdist   = makespan(sol, :solution)
            # sumdist    = totaldist(sol, :solution)
            tau       = sum([ nconstraints(g) for g in chrom ])

            [ relent(colls, n, maxdist, 0.76) ]#* (1 - sig(tau / prod(size(chrom)))) ]
        end
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
                    g = rand() < (1 / 6) ? xmult(g, c) : g
                end

                chrom[r, x, y, t] = g
            end
        end

        chrom
    end
    for _ in 1:popsize ]
end


function select(population::Vector{Chromosome}, fitnesses::Vector{Vector{Float64}}, params::Params, gen, lastimprov)
    m = params.popsize - params.nelites

    # ranking = sort(1 : n, by=i -> fitnesses[i])
    # rank(i) = findfirst(r -> r == i, ranking)

    # amax = 1.2
    # amin = 0.8

    # p(i) = (1 / n) * (amin + (amax - amin) * ((rank(i) - 1) / (n - 1)))

    # weights = map(i -> p(i), 1 : n)

    fitnesses = map(first, fitnesses)
    
    totalfitness = sum(fitnesses)

    pointerdist = totalfitness / m

    start = rand() * pointerdist

    pointers = [ start + i * pointerdist for i in 1 : m -1 ]

    # weights = map(fit -> first(fit) / fitsum, fitnesses)



    # boltzs = map(w -> exp(w / (gen / min(lastimprov^2, gen))), weights)
    # boltzm = sum(boltzs) / length(boltzs)

    # weights = map(b -> b / boltzm, boltzs)

    # sample(population, Weights(weights), m, replace=false)
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


logit(x) = log(x / (1 - x))


depressure(lastimprov, gen) = 1 + max(0.5, min(0, logit(min(lastimprov^2, gen) / gen)))


function reproduce(selection::Vector{Chromosome}, params::Params, paths::Dict{UInt64, Union{Path, Nothing}}, inst::MRMPInstance, lastimprov, gen)
    nchildren = params.popsize - params.nelites

    collect(Iterators.flatten([
        begin
            parents   = sample(selection, 2)

            # mutmut = 1 + exp(min(lastimprov^2, gen) / gen) / (1 + exp(min(lastimprov^2, gen) / gen))

            offspring = crossover(parents..., paths, inst, params.pcross)

            foreach(child -> mutate!(child, params.pmut * depressure(lastimprov, gen)), offspring)

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
            if rand() < 1 / log2(l)
                for c in constraintvals
                    if rand() < (1 / 6)
                        chrom[r, x, y, t] = flip(chrom[r, x, y, t], c)
                    end
                end
            end
        end
    end
end


function solve(paths::Dict{UInt64, Union{Path, Nothing}}, chrom::Chromosome, inst::MRMPInstance)
    chrompaths = getpaths(paths, chrom, inst)
    plan       = map(last, chrompaths)
    newpaths   = map(t -> (t[2], t[3]), filter(p -> p[2], chrompaths))

    (plantosolution(plan), newpaths)
end


function getpaths(paths::Dict{UInt64, Union{Path, Nothing}}, chrom::Chromosome, inst::MRMPInstance)
    n = size(chrom)[1]

    chrompaths = @distributed (vcat) for r in 1:n
        g = genes(r, chrom)
        h = hash((r, g))

        if !haskey(paths, h)
            robot = inst.robots[r]
            path = astar(robot.pos, robot.target, inst; constmat=chrom, r=r)

            (h, true, path)
        else
            path = paths[h]

            (h, false, path)
        end
    end

    chrompaths
end


function solve!(paths::Dict{UInt64, Union{Path, Nothing}}, chrom::Chromosome, inst::MRMPInstance)
    chrompaths = getpaths(paths, chrom, inst)
    plan       = map(last, chrompaths)
    newpaths   = map(t -> (t[2], t[3]), filter(p -> p[2], chrompaths))

    for (gh, path) in newpaths
        paths[gh] = path
    end

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

    rdist = map(r -> rcolls[r] / total, 1:n)

    for x in 1:w, y in 1:h, t in 1:d
        colldist[(x, y, t)] = (maxdist - collisiondist((x, y, t), collvecs)) / maxdist
    end

    # distrib = Biweight(1, 0.25)

    Float64[
        # 2 * log(n) * (1 + rand(dist)) * ((r, (x, y), t) ∈ colls ? ℯ : 1) / total
        (( 1 + rdist[r]) * colldist[x, y, t]) / (n * w * h)
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


function conflicttoconstraints(conflict::Collision)
    if isa(conflict, Clash)
        constraints = map(r -> ClashConstraint(conflict.time, conflict.pos, r), collect(conflict.robots))
    else isa(conflict, Overlap)
        left  = isnothing(conflict.onrobot)  ? nothing : OverlapConstraint(conflict.time, conflict.onsrc, conflict.ondst, conflict.onrobot)
        right = isnothing(conflict.offrobot) ? nothing : OverlapConstraint(conflict.time, conflict.ondst, conflict.offdst, conflict.offrobot)

        constraints = filter(c -> !isnothing(c), [ left, right ])
    end

    Vector{Constraint}(constraints)
end


end
