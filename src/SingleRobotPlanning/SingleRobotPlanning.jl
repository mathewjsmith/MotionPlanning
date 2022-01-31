module SingleRobotPlanning

include("astar.jl")
include("extensionplanner.jl")

using MotionPlanning.Constraints
using MotionPlanning.Collisions
using MotionPlanning.Model
using OffsetArrays

using DataStructures: haskey, isempty

export
    # shortestpaths,
    astar,
    ep,
    manhattandist,
    validmoves,
    neighbours,
    direction
    # sacbs


function manhattandist(start::Pos, target::Pos)
    (xs, ys) = start
    (xt, yt) = target
    float(abs(xs - xt) + abs(ys - yt))
end


function neighbours(cell::Pos, instance::MRMPInstance)
    x, y = cell

    if instance.bounded
        width, height = instance.dims
        filter(n -> !isnothing(n), [
            x - 1 >= 0     ? (x - 1, y) : nothing,
            x + 1 < width  ? (x + 1, y) : nothing,
            y - 1 >= 0     ? (x, y - 1) : nothing,
            y + 1 < height ? (x, y + 1) : nothing,
            (x, y)
        ])
    else
        [ (x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1), (x, y) ]
    end
end


function validmoves(pos::Pos, instance::MRMPInstance; time=1)
    if isdynamic(instance.obstacles)
        nobsts    = length(instance.obstacles)
        obstacles = nobsts > 0 ? instance.obstacles[min(time, nobsts)] : []
        moves     = setdiff(neighbours(pos, instance), obstacles)

        if !isempty(moves) && time > 1 && time <= nobsts
            # check if there is an obstacle moving onto the current position
            onto = findfirst(obst -> obst == pos, obstacles)

            if !isnothing(onto)
                ontodir = direction(instance.obstacles[time - 1][onto], obstacles[onto])
                moves   = filter(dst -> direction(pos, dst) == ontodir, moves)
            end

            # check if there are obstacles moving off of any of the possible destinations
            off = findall(obsts -> obsts ∈ moves, instance.obstacles[time - 1])

            if !isempty(off)
                offdir = Dict([ (instance.obstacles[time - 1][obst], direction(instance.obstacles[time - 1][obst], obstacles[obst])) for obst in off ])
                moves = filter(move -> (move ∉ keys(offdir)) || direction(pos, move) == offdir[move], moves)
            end
        end
        
        moves
    else
        setdiff(neighbours(pos, instance), instance.obstacles)
    end
end


function validmoves(r::Int, pos::Pos, t::Int, constmat::ConstraintMatrix, instance::MRMPInstance)
    moves = neighbours(pos, instance)
    moves = filter(dst -> !isconstrained(r, pos, dst, t, constmat), moves)

    if isdynamic(instance.obstacles)
        nobsts    = length(instance.obstacles)
        obstacles = nobsts > 0 ? instance.obstacles[min(time, nobsts)] : []
        moves     = setdiff(moves, obstacles)

        if !isempty(moves) && time > 1 && time <= nobsts
            # check if there is an obstacle moving onto the current position
            onto = findfirst(obst -> obst == pos, obstacles)

            if !isnothing(onto)
                ontodir = direction(instance.obstacles[time - 1][onto], obstacles[onto])
                moves   = filter(dst -> direction(pos, dst) == ontodir, moves)
            end

            # check if there are obstacles moving off of any of the possible destinations
            off = findall(obsts -> obsts ∈ moves, instance.obstacles[time - 1])

            if !isempty(off)
                offdir = Dict([ (instance.obstacles[time - 1][obst], direction(instance.obstacles[time - 1][obst], obstacles[obst])) for obst in off ])
                moves = filter(move -> (move ∉ keys(offdir)) || direction(pos, move) == offdir[move], moves)
            end
        end
        
        moves
    else
        setdiff(moves, instance.obstacles)
    end

    moves
end


end
