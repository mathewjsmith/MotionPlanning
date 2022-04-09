module CBS

using MotionPlanning.Constraints
using MotionPlanning.Collisions
using MotionPlanning.Model
using MotionPlanning.MultiRobotPlanning.Metrics
using MotionPlanning.SingleRobotPlanning

using DataStructures

export
    cbs


mutable struct Vertex
    parent     :: Union{Vertex, Nothing}
    robot      :: Union{Int, Nothing}
    constraint :: Union{Constraint, Nothing}
    plan       :: Union{Plan, Nothing}
    cost       :: Int
end


"""
    cbs(instance, initialplan)

Perform Conflict Based Search on the given `instance`.
"""
function cbs(instance::MRMPInstance, initialplan::Union{Plan, Nothing}=nothing)
    if isnothing(initialplan)
        initialplan = Plan([ astar(r.pos, r.target, instance) for r in instance.robots ])
    end

    if any(isnothing, initialplan)
        return nothing
    end

    root = Vertex(nothing, nothing, nothing, initialplan, makespan(initialplan, :plan))

    queue = PriorityQueue{Vertex, Int}()
    enqueue!(queue, root, root.cost)

    while !isempty(queue)
        v = dequeue!(queue)
        conflicts = findcollisions(plantosolution(v.plan), instance)

        if isempty(conflicts)
            return (plantosolution(v.plan), getconstraints(v))
        end

        constraints = createconstraints(collect(conflicts), v)

        children = map(constraint -> constrainttovertex(constraint, constraint.robot, v, instance), constraints)

        for w in children
            if !isnothing(w.plan)
                enqueue!(queue, w, w.cost)
            end
        end
    end

    nothing
end


"""
    createconstraints(conflicts, v)

Given a list of `conflicts`, create a constraint per robot per conflict.
"""
function createconstraints(conflicts::Vector{Collision}, v::Vertex)
    conflict = conflicts[1]

    if isa(conflict, Clash)
        robots = isnothing(v.parent) ? collect(conflict.robots) : chooserobots(conflict, v.parent)
        constraints = map(r -> ClashConstraint(conflict.time, conflict.pos, r), robots)
    else isa(conflict, Overlap)
        left  = isnothing(conflict.onrobot) ? 
            nothing : 
            OverlapConstraint(conflict.time, conflict.onsrc, conflict.ondst, conflict.onrobot)

        right = isnothing(conflict.offrobot) ? 
            nothing : 
            OverlapConstraint(conflict.time, conflict.ondst, conflict.offdst, conflict.offrobot)

        constraints = filter(c -> !isnothing(c), [ left, right ])
    end

    setdiff(constraints, getconstraints(v))
end


"""
    chooserobots(conflict, v)

From the robots involved in `conflict`, choose those that do not already have an equivalent constraint expanded in the conflict tree.
"""
function chooserobots(conflict::Clash, v::Vertex)
    occurred = []
    constraint = ClashConstraint(conflict.time, conflict.pos, 1)

    while !isnothing(v.parent)
        if v.constraint == constraint
            push!(occurred, v.robot)
        end

        v = v.parent
    end

    collect(filter(r -> r ∉ occurred, conflict.robots))
end


"""
    constrainttovertex(constraint, robot, parent, instance)
    
From the given `constraint`, create a vertex to be added to the constraint tree.
"""
function constrainttovertex(constraint::Constraint, robot::Int, parent::Vertex, instance::MRMPInstance)
    plan = deepcopy(parent.plan)
    constraints = [ constraint; getconstraints(robot, parent) ]
    newpath = astar(instance.robots[robot].pos, instance.robots[robot].target, instance, constraints)

    if isnothing(newpath)
        plan = nothing
    else
        plan[robot] = newpath
    end

    Vertex(
        parent,
        robot,
        constraint,
        plan,
        isnothing(plan) ? 0 : makespan(plan, :plan)
    )
end


getconfig(time::Int, plan::Plan) = [ path[min(end, time)] for path in plan ]


plantosolution(plan::Plan) = [ getconfig(time, plan) for time in 1:makespan(plan, :plan) ]


"""
    getconstraints(robot, v)

Collect all constraints for the given `robot` in the path through the constraint tree from vertex `v` to the root.
"""
function getconstraints(robot::Int, v::Vertex)
    constraints = Vector{Constraint}()

    while !isnothing(v.parent)
        if v.robot == robot
            push!(constraints, v.constraint)
        end

        v = v.parent
    end

    constraints
end


"""
    getconstraints(v)
    
Collect all constraints in the path through the constraint tree from vertex `v` to the root.
"""
function getconstraints(v::Vertex)
    constraints = Vector{Constraint}()

    while !isnothing(v.parent)
        push!(constraints, v.constraint)
        v = v.parent
    end

    constraints
end


end