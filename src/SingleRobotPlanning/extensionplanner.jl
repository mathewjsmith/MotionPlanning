using MotionPlanning.Collisions
using MotionPlanning.Model
using MotionPlanning.SingleRobotPlanning

using DataStructures


function ep(start::Pos, target::Pos, instance::MRMPInstance; heuristic::Union{Function, Nothing}=nothing, minlength=-1)
    queue = PriorityQueue{Vector{Pos}, Float64}()
    queue[[start]] = 0.0

    # cost = Dict{Pos, Int}()
    # cost[start] = 0

    h(src, dst, target, time) = if isnothing(heuristic)
        manhattandist(src, target)
    else
        heuristic(src, dst, target, time)
    end

    maxpath = 4 * sum(instance.dims)

    while !isempty(queue)
        p = dequeue!(queue)
        v = p[end]
        # vcost = cost[v]
        pcost = length(p)

        if v == target
            return p
        #     while length(p) < minlength
        #         newcost = vcost + 1
        #         if target ∈ validmoves(target, instance; time=newcost)
        #             p = [ p; target ]
        #             vcost = newcost
        #         else
        #             queue = PriorityQueue{Vector{Pos}, Float64}()
        #             for w in validmoves(v, instane; time=newcost)
        #                 q = [ p; w ]
        #                 cost[w] = newcost
        #                 queue[q] = newcost + h(v, w, target, newcost)
        #             end
        #             break
        #         end
        #     end

        #     if length(p) >= minlength
        #         return p
        #     end
        # elseif length(p) > maxpath
        #     return nothing
        end

        newcost = pcost + 1

        for w in validmoves(v, instance; time=newcost)
            # if h(v, w, target, newcost + 1) < Inf && (!haskey(cost, w) || newcost < cost[w] || w == v)
            if h(v, w, target, newcost + 1) < Inf
                q = [ p ; w ]
                # cost[w] = newcost
                queue[q] = newcost + h(v, w, target, newcost)
            end
        end

    end

    nothing
end


function ep(start::Pos, target::Pos, instance::MRMPInstance, constraints::Vector{Constraint})
    h(src, dst, target, time) = createheuristic(src, dst, target, time, constraints)
    ep(start, target, instance; heuristic=h)
end


function createheuristic(src::Pos, dst::Pos, target::Pos, time::Int, constraints::Vector{Constraint})
    penalty = 0

    for constraint in constraints
        if isa(constraint, ClashConstraint)
            if dst == constraint.pos && constraint.time == time
                penalty = Inf
                break
            end
        elseif isa(constraint, OverlapConstraint)
            if src == constraint.src && dst == constraint.dst && time == constraint.time
                penalty = Inf
                break
            end
        end
    end

    manhattandist(src, target) + penalty
end