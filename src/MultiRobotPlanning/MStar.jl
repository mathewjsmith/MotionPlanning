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
    mstar(instance, O, H, ϵ=1.0, initialplan=nothing)

Compute an optimal plan for the given `instance` according to objective `O` using heuristic `H`.

# Arguments
- `O::Objective`: The objective to be minimised, typically `MakeSpan` or `TotalDist`.
- `H::Heuristic`: Typically the `MaxDist` heuristic is used to minimise `MakeSpan`, and `SIC` to minimise `TotalDist`.
- `ϵ::Float`: Set greater than 1.0 to inflate the heuristic. Doing so improves performance at the cost of optimality, creating an ϵ-approximation algorithm.
- `initialplan::Plan`: The initial 1-dimensional path through the configuration-space. If no alternative is provided, the plan induced by having all robots follow their shortest paths is used.

M* is a dynamically coupled algorithm for MRMP. It is a variant of A* search on the configuration-space using subdimensional-expansion. See Wagner and Choset (2015).
"""
function mstar(instance::MRMPInstance, O::Objective, H::Heuristic; ϵ::Float64 = 1.0, initialplan = nothing)
    initialconfig = Config([ r.pos for r in instance.robots ])
    goalconfig    = Config([ r.target for r in instance.robots ])

    shortestpaths = if isnothing(initialplan)
        [ astar(r.pos, r.target, instance) for r in instance.robots ]
    else
        initialplan
    end

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

        # If a vertex has not yet been encountered, or has had new collisions propogated to it since it was last encountered, its outneighbours set needs to be expanded.
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
    expand!(G, v, targets, shortest_paths, instance)

Add outgoing edges from `v` corresponding to its limited neighbours. That is, the combination of configs resulting from robots which collide on their optimal paths choosing from all possible moves, while robots which do not collide take only their optimal move.
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
    expandwithconfig(G, v, wconfig)
    
Expand `v` with an edge to config `w`.
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
    addconfig!(G, config, cost, parent)

Add a node for the given `config` to `G`, and add it to the set of expanded configs.
"""
function addconfig!(G::ConfigGraph, config::Config, cost::Vector{Float64} = [Inf], parent::Union{Vertex, Nothing} = nothing)
    addvertex!(G, config, cost, parent)
end


"""
    backprop!(G, v, queue, wcollset, h)

Propogate newly found collisions to the collision sets of the parents of `v`; add the parents back into the queue.
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
    buildpath(G, v)

After reaching the goal config, reconstruct the optimal path by backtracking through the parents in graph.
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


"""
    findcollidingrobots(srcconfig, dstconfig, instance)

Identify the robots that collide in the transition from `srcconfig` to `dstconfig`.
"""
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
    nextmoves(pos, target, instance)

Return valid moves from `pos`, ordered by distance from `target`.
"""
function nextmoves(pos::Pos, target::Pos, instance::MRMPInstance)
    sort(validmoves(pos, instance), by=move -> manhattandist(move, target))
end


"""
    optimalmove(pos, shortestpath, instance)

Find the optimal move for a robot from its given `pos`, disregarding potential collisions.
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
    movetopath(pos, shortestpath, instance)

Find the move that will take the robot at `pos` closest to its optimal path.
"""
function movetopath(pos::Pos, shortestpath::Path, instance::MRMPInstance)
    closestcells = sort(shortestpath, by = x -> manhattandist(x, pos))
    closestcell = closestcells[1]
    astar(pos, closestcell, instance)[2]
end


end
