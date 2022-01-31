module Collisions

export
    Clash,
    Overlap,
    Collision,
    findcollisions,
    hascollisions,
    findclashes,
    findoverlaps,
    direction

using MotionPlanning.Model

using OffsetArrays


struct Clash
    time   :: Int
    pos    :: Pos
    robots :: Set{Int}    
end


struct Overlap
    time     :: Int
    onsrc    :: Pos
    ondst    :: Pos
    offdst   :: Union{Pos, Nothing}
    onrobot  :: Union{Int, Nothing}
    offrobot :: Union{Int, Nothing}
end


const Collision = Union{Clash, Overlap}


"""
Returns all collisions in the solution.
"""
function findcollisions(solution::Solution, instance::MRMPInstance; ignoreobstacles=false)
    collisions = Set{Collision}()

    for time in 1:(length(solution) - 1)
        currconfig = solution[time]
        nextconfig = solution[time + 1]
        union!(collisions, findcollisions(currconfig, nextconfig, instance; ignoreobstacles=ignoreobstacles, time=time))
    end

    collisions
end


"""
Returns the collisions occurring during the transation from `srconfig` to `dstconfig`.
"""
function findcollisions(srcconfig::Config, dstconfig::Config, instance::MRMPInstance; ignoreobstacles=false, time=0)
    clashes  = findclashes(dstconfig, instance; ignoreobstacles=ignoreobstacles, time=time + 1)
    overlaps = findoverlaps(srcconfig, dstconfig, instance; ignoreobstacles=ignoreobstacles, time=time)
    union(Set{Collision}(), clashes, overlaps)
end


"""
Returns true if there are any collisions in the solution.
"""
function hascollisions(solution::Solution, instance::MRMPInstance; ignoreobstacles=false)
    for time in 1:(length(solution) - 1)
        currconfig = solution[time]
        nextconfig = solution[time + 1]

        if !isempty(findcollisions(currconfig, nextconfig, instance; ignoreobstacles=ignoreobstacles, time=time))
            return true
        end
    end

    false
end


"""
A clash occurs when two robots are trying to move into the same cell.
"""
function findclashes(config::Config, instance::MRMPInstance; ignoreobstacles=false, time=0)
    targets = Dict{Pos, Set{Union{Int64, Nothing}}}()

    for (r, target) in enumerate(config)
        if haskey(targets, target)
            targets[target] = union(targets[target], r)
        else
            targets[target] = Set(r)
        end
    end

    if isdynamic(instance.obstacles) && !ignoreobstacles
        for target in enumerate(instance.obstacles[min(end, time)])
            if haskey(targets, target)
                targets[target] = union(targets[target], nothing)
            end
        end
    end

    [ Clash(time, clash[1], setdiff(Set(clash[2]), Set([nothing]))) for clash in targets if length(clash[2]) > 1 ]
end


"""
An overlap occurs when robot A wants to move into a cell, but there is another robot currently in that cell moving in a different direction.
"""
function findoverlaps(srcconfig::Config, dstconfig::Config, instance::MRMPInstance; ignoreobstacles=false, time=0)
    overlaps = Vector{Overlap}()
    dirmatrix = directionmatrix(srcconfig, dstconfig, instance; ignoreobstacles=ignoreobstacles, time=time)

    for (onrobot, (src, dst)) in enumerate(zip(srcconfig, dstconfig))
        # if there is a robot in the target cell and it is moving in a different direction
        srcdir = get(dirmatrix, src, nothing)
        dstdir = get(dirmatrix, dst, nothing)

        if !isnothing(dstdir) && (srcdir != dstdir)
            offrobot = findfirst(pos -> pos == dst, srcconfig)
            offdst   = isnothing(offrobot) ? nothing : dstconfig[offrobot]

            if dst != offdst
                push!(overlaps, Overlap(time + 1, src, dst, offdst, onrobot, offrobot))
            end
        end
    end

    if !ignoreobstacles && isdynamic(instance.obstacles)
        if length(instance.obstacles) > time
            for (src, dst) in zip(instance.obstacles[time], instance.obstacles[time + 1])
                if dst ∈ srcconfig
                    obstdir = direction(src, dst)

                    if (obstdir != dirmatrix[dst])
                        offrobot = findfirst(pos -> pos == dst, srcconfig)
                        offdst   = dstconfig[offrobot]
                        push!(overlaps, Overlap(time + 1, src, dst, offdst, nothing, offrobot))
                    end
                end
            end
        end
    end

    overlaps
end


"""
Returns the direction of a move from `src` to `dst`.
"""
function direction(src::Pos, dst::Pos)
    a, b = src
    c, d = dst

    if a - c < 0
        :east
    elseif a - c > 0
        :west
    elseif b - d > 0
        :north
    elseif b - d < 0
        :south
    else
        :remain
    end
end


"""
Returns a dictionary mapping positions to the direction that the occupying robot intends to move.
"""
function directionmatrix(src_config::Config, dst_config::Config, instance::MRMPInstance; ignoreobstacles=false, time=0)
    matrix = Dict{Pos, Symbol}()

    for r in 1:length(src_config)
        matrix[src_config[r]] = direction(src_config[r], dst_config[r])
    end

    if isdynamic(instance.obstacles) && !ignoreobstacles
        if length(instance.obstacles) >= time
            for o in 1:length(instance.obstacles[time])
                src = instance.obstacles[time][o]

                if time < length(instance.obstacles)
                    dir = instance.obstacles[time + 1][o]
                    matrix[src] = direction(src, dir)
                else
                    matrix[src] = :remain
                end
            end
        end
    end

    matrix
end


end