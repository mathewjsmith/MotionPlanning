module MStar

export 
    mstar,
    Graph,
    Objectives,
    Heuristics

using MotionPlanning.Collisions
using MotionPlanning.Model
using MotionPlanning.MultiRobotPlanning.Metrics
using MotionPlanning.MultiRobotPlanning.Graph
using MotionPlanning.MultiRobotPlanning.Objectives
using MotionPlanning.MultiRobotPlanning.Heuristics
using MotionPlanning.SingleRobotPlanning

using DataStructures


"""
An exact algorithm for coordinated motion planning. M* is a variant of A* search on the configuration space using subdimensional-expansion.
"""
function mstar(instance::MRMPInstance, O::Objective, H::Heuristic; ϵ::Float64 = 1.0)
    initialconfig = Config([ r.pos for r in instance.robots ])
    goalconfig    = Config([ r.target for r in instance.robots ])
    shortestpaths = [ astar(r.pos, r.target, instance) for r in instance.robots ]

    f(v::Vertex, w::Vertex)::Cost = weight(O, v.config, w.config)
    h(v::Vertex)::Cost = ϵ .* heuristic(H, v.config, goalconfig)

    G = ConfigGraph()

    start = addconfig!(G, initialconfig, [0.0])

    queue = PriorityQueue{Vertex, Vector{Float64}}()
    enqueue!(queue, start, start.cost)

    while !isempty(queue)
        v = dequeue!(queue)

        if v.config == goalconfig
            path = buildpath(G, v)
            return (map(u -> u.config, path), G, path)
        end

        # If a vertex has not yet been encountered, or has had new collisions propogated to it since it was last encountered, its outNeighbours set needs to be expanded.
        if !v.expanded
            expand!(G, v, goalconfig, shortestpaths, instance)
        end

        for w in outneighbours(G, v)
            vwcollisions = findcollidingrobots(v.config, w.config, instance)
            w.collset    = collsetunion!(w, vwcollisions)

            backsetappend!(w, v)
            backprop!(G, v, queue, w.collset, h)

            newcost = v.cost .+ f(v, w)

            # If there are no collisions from v to w, and v provides the cheapest route to w encountered so far, set v to be the parent of w and add w to the queue.
            if isempty(vwcollisions) && (newcost < w.cost)
                w.cost   = newcost
                queue[w] = w.cost .+ h(w)
                setparent!(w, v)
            end
        end
    end

    nothing
end


"""
Adds outgoing edges from `v` corresponding to its limited neighbours. That is, the combination of configs resulting from robots which collide on their optimal paths choosing from all possible moves, while robots which do not collide take only their optimal move.
"""
function expand!(G::ConfigGraph, v::Vertex, targets::Config, shortest_paths::Array{Array{Pos, 1}, 1}, instance::MRMPInstance)
    moves = [
        if i in v.collset
            # Robots in the collision set consider robots.
            nextmoves(pos, target, instance)
        else
            # Other robots continue along their optimal path.
            [ optimalmove(pos, shortest_paths[i], instance) ]
        end
        for (i, (pos, target)) in enumerate(zip(v.config, targets))
    ]

    # Create a config from each combinations of individual robot moves.
    for config in Iterators.product(moves...)
        expandwithconfig(G, v, collect(config))
    end

    v.expanded = true
end


"""
Expands v with an edge to config w.
"""
function expandwithconfig(G::ConfigGraph, v::Vertex, wconfig::Config)
    if !hasvertex(G, wconfig)
        # if a vertex for w does not exist, create it.
        w = addconfig!(G, wconfig, [Inf])
    else
        # Otherwise get the corresponding vertex for the config.
        w = getvertex(G, wconfig)
    end

    addedge(v, w)
end


"""
Adds a node for the given config to the graph and adds it to the set of expanded configs.
"""
function addconfig!(G::ConfigGraph, config::Config, cost::Vector{Float64} = [Inf], parent::Union{Vertex, Nothing} = nothing)
    addvertex!(G, config, cost, parent)
end


"""
Propogates newly found collisions to the collision sets of a vertex's parents; adds the parents back into the queue.
"""
function backprop!(G::ConfigGraph, v::Vertex, queue::PriorityQueue{Vertex, Vector{Float64}}, wcollset::Set{UInt64}, h::Function)
    if !issubset(wcollset, v.collset)
        v.collset  = union(wcollset, v.collset)
        v.expanded = false

        if !(v in keys(queue))
            queue[v] = v.cost .+ h(v)
        end

        for u in backset(G, v)
            backprop!(G, u, queue, v.collset, h)
        end
    end
end


"""
After reaching the goal config, reconstructs the optimal path by working back through the parents in graph.
"""
function buildpath(G::ConfigGraph, v::Vertex)
    parent = getparent(G, v)
    path   = [ v ]

    while !isnothing(parent)
        path   = [ parent; path ]
        parent = getparent(G, parent)
    end

    path
end


function findcollidingrobots(srcconfig::Config, dstconfig::Config, instance::MRMPInstance)
    collisions = findcollisions(srcconfig, dstconfig, instance)

    Set{UInt64}(Iterators.flatten(map(
        coll ->
            if isa(coll, Clash)
                collect(coll.robots)
            elseif isa(coll, Overlap)
                [coll.onrobot, coll.offrobot]
            end,
        collect(collisions)
    )))
end


"""
Returns valid moves from the given position, ordered by distance from the target.
"""
function nextmoves(pos::Pos, target::Pos, instance::MRMPInstance)
    sort(validmoves(pos, instance), by=move -> manhattandist(move, target))
end


"""
Finds the optimal move for a robot from its given position, disregarding potential collisions.
"""
function optimalmove(pos::Pos, shortestpath::Path, instance::MRMPInstance)
    pathindex = findfirst(x -> x == pos, shortestpath)

    if pos == shortestpath[end]
        # if the robot is on its target, don't move.
        return pos
    elseif isnothing(pathindex)
        # if the robot has moved off of its optimal path, find the best way back onto the path.
        return movetopath(pos, shortestpath, instance)
    else
        # if the robot is on the optimal path, take the next step.
        return shortestpath[pathindex + 1]
    end
end


"""
Finds the move that will take the robot at pos closest to its optimal path.
"""
function movetopath(pos::Pos, shortestpath::Path, instance::MRMPInstance)
    closestcells = sort(shortestpath, by = x -> manhattandist(x, pos))
    closestcell = closestcells[1]
    astar(pos, closestcell, instance)[2]
end


end
