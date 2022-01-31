module Metrics

using MotionPlanning.Constraints
using MotionPlanning.Model
using MotionPlanning.MultiRobotPlanning.Graph
using MotionPlanning.SingleRobotPlanning

export
    makespan,
    totaldist,
    κ


function makespan(solution::Union{Plan, Solution}, type::Symbol)
    if type == :plan
        maximum([ length(path) for path in solution ])
    elseif type == :solution
        length(solution)
    end
end


function totaldist(solution::Union{Plan, Solution}, type::Symbol)
    nmoves = 0

    if type == :plan
        for path in solution
            for time in 1:length(path) - 1
                curr = path[time]
                next = path[time + 1]

                if next != curr
                    nmoves += 1
                end
            end
        end
    elseif type == :solution
        for r in 1:length(solution[1])
            for time in 1:length(solution) - 1
                curr = solution[time][r]
                next = solution[time + 1][r]

                if next != curr
                    nmoves += 1
                end
            end
        end
    end

    nmoves
end


function κ(G::ConfigGraph, path::Vector{Vertex})
    kappas = []

    for v in path
        moves = Dict{Int, Set{Symbol}}()

        for w in outneighbours(G, v)
            for (r, dst) in enumerate(w.config)
                dir = direction(v.config[r], dst)

                if haskey(moves, r)
                    union!(moves[r], Set([dir]))
                else
                    moves[r] = Set([dir])
                end
            end
        end

        push!(kappas, length(filter(k -> k > 1, map(p -> length(p[2]), collect(moves)))))
    end

    (minimum(kappas), maximum(kappas), sum(kappas) / length(kappas))
end


function κ(solution::Solution, constraints::Vector{Constraint}, instance::MRMPInstance)
    adjacencies = Dict{Config, Vector{Config}}()

    for (time, config) in enumerate(solution[1:end - 1])
        adjacencies[config] = []

        moves = [
            begin
                dsts = validmoves(pos, instance; time=time + 1)

                if any(map(c -> any(map(dst -> matchconstraint(r, time + 1, pos, dst, c), dsts)), constraints))
                    dsts
                else
                    [ solution[time + 1][r] ]
                end
            end
            for (r, pos) in enumerate(config)
        ]

        for adjconfig in Iterators.product(moves...)
            push!(adjacencies[config], collect(adjconfig))
        end
    end

    kappas = []

    for (v, ws) in pairs(adjacencies)
        moves = Dict{Int, Set{Symbol}}()

        for w in ws
            for (r, dst) in enumerate(w)
                dir = direction(v[r], dst)

                if haskey(moves, r)
                    union!(moves[r], Set([dir]))
                else
                    moves[r] = Set([dir])
                end
            end
        end

        push!(kappas, length(filter(k -> k > 1, map(p -> length(p[2]), collect(moves)))))
    end

    (minimum(kappas), maximum(kappas), sum(kappas) / length(kappas))
end


function matchconstraint(robot::Int, time::Int, src::Pos, dst::Pos, constraint::Constraint)
    if isa(constraint, ClashConstraint)
        constraint.robot == robot && constraint.time == time && constraint.pos == dst
    elseif isa(constraint, OverlapConstraint)
        constraint.robot == robot && constraint.time == time && constraint.src == src && constraint.dst == dst
    end
end


# function κ(G::ConfigGraph)
#     kappas = [ length(v.collset) for v in values(G) ]
#     min = minimum(kappas)
#     max = maximum(kappas)
#     mean = sum(kappas) / length(kappas)
#     (min, max, mean)
# end


end