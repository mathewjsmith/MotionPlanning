using MotionPlanning.Constraints
using MotionPlanning.Collisions
using MotionPlanning.Model
using MotionPlanning.SingleRobotPlanning

using DataStructures


const TPos = Tuple{Int, Pos}

gettime(pos::TPos) = pos[1]
getpos(pos::TPos)  = pos[2]


function astar(robot::Robot, instance::MRMPInstance; heuristic::Union{Function, Nothing}=nothing)
    astar(robot.pos, robot.target, instance; heuristic=heuristic)
end


function astar(
    start     :: Pos, 
    target    :: Pos, 
    instance  :: MRMPInstance; 
    heuristic :: Union{Function, Nothing}=nothing,
    constmat  :: Union{Nothing, ConstraintMatrix}=nothing,
    r         :: Int=0
)
    parents = Dict{TPos, Union{TPos, Nothing}}()
    # costs   = Dict{Pos, Int}()
    queue   = PriorityQueue{TPos, Float64}()

    maxlength = 4 * sum(instance.dims)

    h(src, dst, target, cost) = if isnothing(heuristic)
        manhattandist(dst, target)
    else
        heuristic(src, dst, target, cost)
    end

    queue[(1, start)]   = 0
    # costs[start]      = 1
    parents[(1, start)] = nothing

    while !isempty(queue)
        (t, v) = dequeue!(queue)
        # vcost = costs[v]

        if v == target
            return buildpath((t, v), parents)
        elseif t > maxlength
            return nothing
        end

        tt = t + 1

        adjacents = if tt <= maxlength
            if isnothing(constmat)
                validmoves(v, instance; time=tt)
            else
                validmoves(r, v, tt, constmat, instance)
            end
        else
            []
        end

        for w in adjacents
            # if (
                # h(v, w, target, tt) < Inf # && (
                #     !haskey(costs, w) || 
                #     tt < costs[w] || 
                #     (w == v && tt <= maxlength) # && vcost <= costs[parents[v]])
                # )
            # )
                # if w == v && v != adjacents[1] # only consider a `remain` move when it is the best available
                #     continue
                # end

                # costs[w]   = tt
                queue[(tt, w)] = tt + h(v, w, target, tt)
                parents[(tt, w)] = (t, v)

                # if w != v
                #     parents[w] = v
                # end
            # end
        end
    end

    nothing
end


function buildpath(v::TPos, parents::Dict{TPos, Union{TPos, Nothing}})
    path = Path([v[2]])
    curr = v
    prev = parents[v]

    while !isnothing(prev)
        curr = prev
        prev = parents[curr]

        path = [ curr[2]; path ]
    end

    path
end


function astar(
    start       :: Pos, 
    target      :: Pos, 
    instance    :: MRMPInstance, 
    constraints :: Vector{Constraint}
)
    h(src, dst, target, time) = createheuristic(src, dst, target, time, constraints)
    astar(start, target, instance; heuristic=h)
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

    manhattandist(dst, target) + penalty
end