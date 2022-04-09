module Model

using OffsetArrays

export
    Pos,
    Config,
    Solution,
    Path,
    Plan,
    getconfig,
    plantosolution,
    solutiontoplan,
    Robot,
    StaticObstacles,
    DynamicObstacles,
    Obstacle,
    isdynamic,
    Cell,
    MRMPInstance,
    iscomplete


"""
A point `(x, y)` in discrete 2D space.
"""
const Pos = Tuple{Int64, Int64}


"""
A state of an MRMP system. Index `i` of a config is the position of robot `i` in that config.
"""
const Config = Vector{Pos}


"""
A solution to an MRMP instance in the form of a sequence of configs.
"""
const Solution = Vector{Config}


"""
A path for a robot in the form of a sequence of positions.
"""
const Path = Vector{Pos}


"""
A solution to an MRMP instance in the form of a list consisting of a path for each robot.
"""
const Plan = Vector{Path}


"""
    makespan(plan)

Compute the makespan of a solution to an MRMP instance given as a `plan`.
"""
makespan(plan::Plan) = maximum(length(path) for path in plan)


"""
    getconfig(time, plan)

Get the config of a solution to an MRMP instance at a specified `time` when the solution is given as a `plan`.
"""
getconfig(time::Int, plan::Plan) = [ path[min(end, time)] for path in plan ]


"""
    plantosolution(plan)

Convert the given `plan::Plan` (list of robot paths) into a `Solution` (list of configs).
"""
plantosolution(plan::Plan) = [ getconfig(time, plan) for time in 1:makespan(plan) ]


"""
    solutiontoplan(sol)

Convert the given `sol::Solution` (list of configs) into a `Plan` (list of robot paths).
"""
solutiontoplan(sol::Solution) = begin
	n = length(sol[1])
	l = length(sol)
	
	plan = Vector{Tuple{Int, Int}}[]
	
	for r in 1:n
		path = []

		for t in 1:l
			push!(path, sol[t][r])
		end

		push!(plan, path)
	end

	plan
end


mutable struct Robot
    id     :: Int64
    pos    :: Pos
    target :: Pos
end


const Obstacle = Pos


const StaticObstacles = Vector{Obstacle}


const DynamicObstacles = Vector{StaticObstacles}


const Obstacles = Union{StaticObstacles, DynamicObstacles}


"""
    isdynamic(obsts)

Determine if the given set of obstacles `obsts` is dynamic.
"""
isdynamic(obsts::Obstacles) = isa(obsts, DynamicObstacles)


"""
Union of the possible states of each position in the grid of an MRMP instance.
"""
const Cell = Union{Nothing, Obstacle, Robot}


"""
    MRMPInstance(name, robots, obstacles[, dims, bounded])

An instance of the multi-robot motion planning problem (MRMP). Characterised by a list of `robots`, each with a unique start and target position, and a set of `obstacles` which may be static or dynamic. `bounded` determines whether or not robots are allowed to move outside the specified `dims`.
"""
mutable struct MRMPInstance
    name      :: String
    robots    :: Vector{Robot}
    obstacles :: Union{StaticObstacles, DynamicObstacles}
    dims      :: Tuple{Int64, Int64}
    bounded   :: Bool
end


function MRMPInstance(name::String, robots::Vector{Robot}, obstacles::Obstacles)
    width, height = dimensions(robots, obstacles)
    MRMPInstance(name, robots, obstacles, (width, height), true)
end


"""
    dimensions(robots, obstacles)

Derive the dimensions of an MRMP instance by finding the minimum and maximum positions across the start and target positions of the `robots`, plus the positions of `obstacles`.
"""
function dimensions(robots, obstacles)
    width = 0
    height = 0

    for r in robots
        x, y = max(r.pos[1], r.target[1]), max(r.pos[2], r.target[2])
        width = max(x, width)
        height = max(y, height)
    end

    if isdynamic(obstacles)
        for t in obstacles
            for o in t
                x, y = o
                width  = max(x, width)
                height = max(y, height)
            end
        end
    else
        for o in obstacles
            x, y = o
            width  = max(x, width)
            height = max(y, height)
        end
    end

    (width + 1, height + 1)
end


Base.:(==)(a::Robot, b::Robot) = a.id == b.id


Base.hash(r::Robot) = Base.hash(r.id)


"""
    iscomplete(robot)

Determine if the `robot` has reached its target.
"""
iscomplete(robot::Robot) = robot.pos == robot.target


end
