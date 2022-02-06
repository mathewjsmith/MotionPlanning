module Model

using OffsetArrays
using JSON2

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
    iscomplete,
    readinstance,
    readinstances,
    random,
    instance_to_json,
    writeinstance,
    writesolution,
    readsolution,
    Benchmark,
    writebenchmark,
    readbenchmark


const Pos = Tuple{Int64, Int64}


const Config = Vector{Pos}


const Solution = Vector{Config}


const Path = Vector{Pos}


const Plan = Vector{Path}

makespan(plan::Plan) = maximum(length(path) for path in plan)

getconfig(time::Int, plan::Plan) = [ path[min(end, time)] for path in plan ]

plantosolution(plan::Plan) = [ getconfig(time, plan) for time in 1:makespan(plan) ]

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


const Obstacle = Tuple{Int64, Int64}


const StaticObstacles = Vector{Obstacle}


const DynamicObstacles = Vector{StaticObstacles}


const Obstacles = Union{StaticObstacles, DynamicObstacles}


isdynamic(obsts::Obstacles) = isa(obsts, DynamicObstacles)


const Cell = Union{Nothing, Obstacle, Robot}


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


iscomplete(robot::Robot) = robot.pos == robot.target


end
