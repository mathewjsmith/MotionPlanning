module Shadoks

export
    shadoks

using MotionPlanning.Collisions
using MotionPlanning.Model
using MotionPlanning.MultiRobotPlanning.Metrics
using MotionPlanning.SingleRobotPlanning
using MotionPlanning.Utils

using InvertedIndices
using DataStructures


function shadoks(instance::MRMPInstance)
    inst = deepcopy(instance)

    n = length(instance.robots)

    if isa(inst.obstacles, StaticObstacles)
        inst.obstacles = DynamicObstacles([ inst.obstacles ])
    end
	
	inst.bounded = false

    mindepth = 3
    network  = storagenetwork(inst, mindepth)
    matching = match(inst.robots, network)
	box      = boundingbox(inst.dims, mindepth)

	startordering(r) = depth(r.pos, box, inst)
    startpriority = sort(collect(enumerate(inst.robots)), by=p -> startordering(p[2]))
	
	targordering(r) = depth(r.target, box, inst)
	targpriority = sort(collect(enumerate(inst.robots)), by=p -> targordering(p[2]), rev=true)

    paths = Dict{Int, Path}()

    inst.obstacles[1] = [ repeat([(0, 0)], length(inst.robots)); inst.obstacles[1] ]

    for (i, r) in enumerate(inst.robots)
        inst.obstacles[1][i] = r.pos
    end

    for (i, r) in startpriority
        for t in 1:length(inst.obstacles)
            inst.obstacles[t] = inst.obstacles[t][Not(i)]
        end

        path = astar(r.pos, matching[i], inst)

        if isnothing(path)
			return nothing
		else
            paths[i] = path
            addobstacles!(r, i, path, inst)
        end
    end

    for (i, r) in targpriority
        for t in 1:length(inst.obstacles)
            inst.obstacles[t] = inst.obstacles[t][Not(i)]
        end

        path = astar(r.pos, r.target, inst)

        if isnothing(path)
            return nothing
        else
            paths[i] = path
            addobstacles!(r, i, path, inst)
        end
    end

    sol = plantosolution(collect(values(sort(paths, by=p -> p[1]))))

    # colls = findcollisions(sol, instance; ignoreobstacles=false)

    # if !isempty(colls)
    #     ordering = r -> manhattandist(r.pos, r.target)

    #     robots = collect(Iterators.flatten([
    #         if isa(coll, Clash)
    #             coll.robots
    #         elseif isa(coll, Overlap)
    #             filter(!isnothing, [ coll.onrobot, coll.offrobot ])
    #         end
    #         for coll in colls
    #     ]))

    #     priority = sort(map(r -> inst.robots[r], robots), by=ordering)

    #     for r in priority
    #         for t in 1:length(inst.obstacles)
    #             inst.obstacles[t] = inst.obstacles[t][Not(r.id)]
    #         end

    #         path = ep(r.pos, r.target, inst)

    #         if isnothing(path)
    #             return nothing
    #         else
    #             paths[r.id] = path
    #             addobstacles!(r, path, inst)
    #         end
    #     end

    #     sol = plantosolution(collect(values(sort(paths, by=p -> p[1]))))
    # end

    sol
end


function isoutside(pos, box)
    x, y = pos
    xmin, ymin = box[1]
    xmax, ymax = box[2]
    x < xmin || x > xmax || y < ymin || y > ymax
end


boundingbox(dims, mindepth) = ((-mindepth, -mindepth), dims .+ (mindepth -1))


function depth(pos::Pos, box::Tuple{Pos, Pos}, instance::MRMPInstance)
    cost  = Dict{Tuple{Int, Int}, Int}()
    cost[pos] = 0

    queue = PriorityQueue{Pos, Int}()
    queue[pos] = 0

    while !isempty(queue)
        v = dequeue!(queue)
        v_cost = cost[v]

        if isoutside(v, box)
            return v_cost
        end

        new_cost = v_cost + 1

        for w in validmoves(v, instance; time=new_cost)
            if !haskey(cost, w) || new_cost < cost[w]
                cost[w] = new_cost
                queue[w] = new_cost
            end
        end
    end
end


function storagenetwork(instance, mindepth=3)
	w, h = instance.dims
	box = boundingbox(instance.dims, mindepth)
	
	top = [ 
		(x, y) 
		for x in box[1][1]:2:box[2][1], 
			y in box[1][2]:-1:box[1][2]-ceil(Int, h /2)
	]
	bottom = [
		(x, y)
		for x in box[1][1]:2:box[2][1], 
			y in box[2][2]:box[2][2]+ceil(Int, h /2)	
	]
	
	left = [ 
		(x, y) 
		for y in box[1][2]:2:box[2][2], 
			x in box[1][1]:-1:box[1][1]-ceil(Int, w /2)
	]
	
	right = [ 
		(x, y) 
		for y in box[1][2]:2:box[2][2], 
			x in box[2][1]:box[2][1]+ceil(Int, w /2)
	]
	
	 vec([ top; bottom; left; right ])
end


function match(robots::Vector{Robot}, network::Vector{Pos})
    costmatrix = [ manhattandist(r.pos, mid) + manhattandist(r.target, mid) for mid in network, r in robots ]
    assignment, _ = hungarian(costmatrix')
    Dict{Int, Pos}([ (r, network[a]) for (r, a) in enumerate(assignment) ])
end


function addobstacles!(robot::Robot, i::Int, path::Vector{Pos}, instance::MRMPInstance)
	pathlength = length(path)
	obstlength = length(instance.obstacles)

	if obstlength < pathlength
		for _ in (obstlength + 1) : pathlength
			push!(instance.obstacles, instance.obstacles[end])
		end
	end

	for (t, pos) in enumerate(path)
        instance.obstacles[t] = [ instance.obstacles[t][1:i - 1]; pos; instance.obstacles[t][i:end]]
	end

	if pathlength < obstlength
		for t in (pathlength + 1) : obstlength
            instance.obstacles[t] = [ instance.obstacles[t][1:i - 1]; path[end]; instance.obstacles[t][i:end]]
		end
	end
end


function plantosolution(plan::Plan)
    cost = makespan(plan, :plan)
    paths = map(path -> pad(path, cost), plan)
    solution = Solution(repeat([[]], cost))

    for path in paths, (t, pos) in enumerate(path)
        solution[t] = [ solution[t]; pos ]
    end

    solution
end


function pad(path::Path, makespan::Int)
    if length(path) < makespan
        [ path; repeat([path[end]], makespan - length(path)) ]
    else
        path
    end
end


end