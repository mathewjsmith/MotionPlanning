using MotionPlanning.Constraints
using MotionPlanning.Collisions
using MotionPlanning.Model
using MotionPlanning.SingleRobotPlanning

using DataStructures


const TPos = Tuple{Int, Pos}

gettime(pos::TPos) = pos[1]
getpos(pos::TPos)  = pos[2]


"""
    astar(start, target, instance, heuristic=nothing, constmat=nothing, r=nothing)

Perform A* search to find a shortest path from `start` to `target` in the given MRMP instance.

# Arguments
- heuristic: function with signature `heuristic(src, dst, target, cost) -> Float`. Defaults to manhattan distance.
- constmat: constraint matrix---a 4d boolean array such that if `constmat[r, x, y, t]` is true, robot `r` cannot move onto `(x, y)` at time `t`.
- r: robot id, used to find relevant constraints when a constraint matrix is provided.
"""
function astar(
    start     :: Pos, 
    target    :: Pos, 
    instance  :: MRMPInstance; 
    heuristic :: Union{Function, Nothing}=nothing,
    constmat  :: Union{Nothing, ConstraintMatrix}=nothing,
    r         :: Int=0
)
    parents = Dict{TPos, Union{TPos, Nothing}}()
    queue   = PriorityQueue{TPos, Float64}()

    maxlength = 4 * sum(instance.dims)

    h(src, dst, target, cost) = if isnothing(heuristic)
        manhattandist(dst, target)
    else
        heuristic(src, dst, target, cost)
    end

    queue[(1, start)]   = 0
    parents[(1, start)] = nothing

    while !isempty(queue)
        (t, v) = dequeue!(queue)

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
            queue[(tt, w)] = tt + h(v, w, target, tt)
            parents[(tt, w)] = (t, v)
        end
    end

    nothing
end


function astar(robot::Robot, instance::MRMPInstance; heuristic::Union{Function, Nothing}=nothing)
    astar(robot.pos, robot.target, instance; heuristic=heuristic)
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


"""
    buildpath(v, parents)

Upon expanding the target position, reconstruct the shortest fast by backtracking through parents until the start position is reached.
"""
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


"""
    createheuristic(src, dst, target, time, constraints)

Given a list of `constraints`, create a modified version of the manhattan distance heuristic where vertices corresponding to constraints have a cost of `Inf`.
"""
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