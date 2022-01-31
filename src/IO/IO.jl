module IO

using MotionPlanning.Model

using JSON
using JSON2
using Glob

export
    readinstance,
    readinstances,
    writeinstance,
    instancetojson


function readinstance(filename::String)
    d = open(filename) do file
        JSON.parse(read(file, String))
    end

    name = d["name"]

    robots = map(
        r -> readrobot(flatten(r)...),
        enumerate(zip(d["starts"], d["targets"]))
    )

    obstacles = map(
        obst -> readobstacle(obst),
        d["obstacles"]
    )

    MRMPInstance(name, robots, StaticObstacles(obstacles))
end


function readinstances(directory)
    files = glob(string(directory, "/*/*.instance.json"))
    readinstance.(files)
end


function readrobot(id::Int64, start_Pos::Array{Any}, target_Pos::Array{Any})
    Robot(id, (start_Pos[1], start_Pos[2]), (target_Pos[1], target_Pos[2]))
end


function readobstacle(Pos::Array{Any})
    Obstacle((Pos[1], Pos[2]))
end


function flatten(x::Tuple{Int64, Tuple{Any, Any}})
    (x[1], x[2][1], x[2][2])
end


function writeinstance(instance, filename)
    json = instancetojson(instance)
    open(filename, "w") do file
        write(file, json)
    end
end


function instancetojson(instance)
    dict = Dict()
    dict["name"] = instance.name
    dict["obstacles"] = [ o.pos for o in instance.obstacles ]
    dict["starts"] = [ r.pos for r in instance.robots ]
    dict["targets"] = [ r.target for r in instance.robots ]
    JSON2.write(dict)
end


end
