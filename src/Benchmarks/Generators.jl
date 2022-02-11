module Generators

using MotionPlanning.Collisions
using MotionPlanning.IO
using MotionPlanning.Model
using MotionPlanning.SingleRobotPlanning

using StatsBase
using Statistics

export
    generate,
    density,
    spread,
    shortestpathcollisions


density(inst::MRMPInstance) = length(inst.robots) / prod(inst.dims)


function spread(inst::MRMPInstance)
    minsqdist = map(r -> nearestneighbour(r, inst.robots)^2, inst.robots)

    mean = sum(minsqdist) / length(inst.robots)

    stdm(minsqdist, mean)
end


function nearestneighbour(robot::Robot, robots::Vector{Robot})
    robots = setdiff(robots, Set([robot]))
    minimum(manhattandist(robot.pos, r.pos) for r in robots) 
end


function shortestpathcollisions(inst::MRMPInstance)
    solution = plantosolution([astar(r.pos, r.target, inst) for r in inst.robots ])
    
    length(findcollisions(solution, inst))
end


function generate(n::Int, dims::Tuple{Int, Int}, clusterfactor::Float64, id::String)
    width, height = dims

    positions = reduce(vcat, [ (x, y) for x in 0 : width - 1, y in 0 : height - 1 ])

    clusterpoints = sample(positions, ceil(Int, n / 5), replace=false)
    maxdist       = manhattandist((0, 0), dims)
    proximities   = [ maxdist - minimum(manhattandist(pos, point) for point in clusterpoints) for pos in positions ]
    proxsum       = sum(proximities)

    θ = clusterfactor

    weights = map(prox -> (1 - θ) * (1 / n) + θ * (prox^(sum(dims)) / proxsum^sum(dims)), proximities)

    starts  = sample(positions, Weights(weights), n, replace=false)
    targets = sample(positions, Weights(weights), n, replace=false)

    robots = [ Robot(i, starts[i], targets[i]) for i in 1 : n ]

    MRMPInstance(
        id,
        robots,
        StaticObstacles(),
        dims,
        true
    )
end


function neighbours(pos::Pos, dims::Tuple{Int, Int})
    x, y = pos

    width, height = dims

    filter(n -> !isnothing(n), [
        x - 1 >= 0     ? (x - 1, y) : nothing,
        x + 1 < width  ? (x + 1, y) : nothing,
        y - 1 >= 0     ? (x, y - 1) : nothing,
        y + 1 < height ? (x, y + 1) : nothing,
    ])
end


function generate()
    spreads    = [0.0, 0.33, 0.66, 1.0]
    spreadname = Dict([(0.0, "000"), (0.33, "033"), (0.66, "066"), (1.0, "100")])

    makename(i, n, dims, spread) = "rand_$(lpad(i, 3, "0"))_n$(lpad(n, 3, "0"))_d$(dims[1])x$(dims[2])_c$(spreadname[spread])"

    # i = 0

    # dims = (4, 4)
    # ns   = 4:15

    # for n in ns
    #     for spread in spreads
    #         for _ in 1:5
    #             name     = makename(i, n, dims, spread)
    #             inst     = generate(n, dims, spread, name)
    #             filename = "instances/new/$(name).instance.json"

    #             writeinstance(inst, filename)

    #             i += 1
    #         end
    #     end
    # end

    # dims = (8, 8)
    # ns   = [ collect(4:15); collect(16:4:44) ]

    # for n in ns
    #     for spread in spreads
    #         for _ in 1:5
    #             name     = makename(i, n, dims, spread)
    #             inst     = generate(n, dims, spread, name)
    #             filename = "instances/new/$(name).instance.json"

    #             writeinstance(inst, filename)

    #             i += 1
    #         end
    #     end
    # end

    i = 1120
    dims = (8, 8)
    ns   = 17:19

    for n in ns
        for spread in spreads
            for _ in 1:5
                name     = makename(i, n, dims, spread)
                inst     = generate(n, dims, spread, name)
                filename = "instances/new/$(name).instance.json"

                writeinstance(inst, filename)

                i += 1
            end
        end
    end

end


end
