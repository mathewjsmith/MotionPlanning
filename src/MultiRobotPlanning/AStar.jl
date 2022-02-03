module AStar

export 
    plan

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
function plan(instance::MRMPInstance)
    start  = Config([ r.pos for r in instance.robots ])
    target = Config([ r.target for r in instance.robots ])
    
    parent = Dict{Config, Union{Config, Nothing}}()
    parent[start] = nothing

    shortestpaths = [ astar(r.pos, r.target, instance) for r in instance.robots ]

    h(v::Config) = maximum([ manhattandist(pos, dst) for (pos, dst) in zip(v, target)])

    cost = Dict{Config, Int}()
    cost[start] = 1

    queue = PriorityQueue{Config, Int}()
    enqueue!(queue, start, 1)

    while !isempty(queue)
        v = dequeue!(queue)

        if v == target
            return buildpath(v, parent)
        end

        for w in outneighbours(v, target, instance)
            collisionfree = isempty(findcollisions(v, w, instance))

            newcost = cost[v] + 1

            # If there are no collisions from v to w, and v provides the cheapest route to w encountered so far, set v to be the parent of w and add w to the queue.
            if collisionfree && (!haskey(cost, w) || (newcost < cost[w]))
                cost[w]   = newcost
                queue[w]  = newcost + h(w)
                parent[w] = v
            end
        end
    end

    nothing
end


function outneighbours(v::Config, targets::Config, instance::MRMPInstance)
    moves = [ nextmoves(pos, target, instance) for (pos, target) in zip(v, targets) ]
    map(collect, collect(Iterators.product(moves...)))
end



"""
After reaching the goal config, reconstructs the optimal path by working back through the parents in graph.
"""
function buildpath(v::Config, parents::Dict{Config, Union{Config, Nothing}})
    parent = parents[v]
    path   = [ v ]

    while !isnothing(parent)
        path   = [ [ parent ]; path ]
        parent = parents[parent]
    end

    path
end


"""
Returns valid moves from the given position, ordered by distance from the target.
"""
function nextmoves(pos::Pos, target::Pos, instance::MRMPInstance)
    sort(validmoves(pos, instance), by=move -> manhattandist(move, target))
end


end
