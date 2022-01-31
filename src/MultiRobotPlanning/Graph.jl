module Graph

using MotionPlanning.Model
using MotionPlanning.MultiRobotPlanning.Objectives

export
    Vertex,
    ConfigGraph,
    nvertices,
    addvertex!,
    addedge,
    collsetunion!,
    backsetappend!,
    isexpanded,
    getparent,
    backset,
    outneighbours,
    setparent!,
    hasvertex,
    getvertex


mutable struct Vertex
    id            :: Int64
    config        :: Config
    cost          :: Cost
    collset       :: Set{UInt64}
    backset       :: Set{UInt64}
    parent        :: Union{UInt64, Nothing}
    expanded      :: Bool
    outneighbours :: Set{UInt64}
end

Base.:(==)(a::Vertex, b::Vertex) = a.config == b.config

Base.hash(v::Vertex) = Base.hash(v.config)

Base.string(v::Vertex) = "Vertex { 
    id: $(v.id), cost: $(v.cost), 
    parent: $(v.parent), 
    n_collisions: $(length(v.collset)), 
    n_outneighbours: $(length(v.outneighbours)), 
    expanded: $(v.expanded)
}"


const ConfigGraph = Dict{UInt64, Vertex}

Base.string(G::ConfigGraph) = [ string(v) for v in values(G)]


function addvertex!(G::ConfigGraph, config::Config, cost::Cost = [ Inf ], parent::Union{Vertex, Nothing} = nothing)
    id = nvertices(G) + 1

    v = Vertex(
        id,
        config,
        cost,
        Set{UInt64}(),
        Set{UInt64}(),
        isnothing(parent) ? nothing : hash(parent),
        false,
        Set{UInt64}()
    )

    G[hash(v)] = v

    v
end


nvertices(G::ConfigGraph) = length(G)


addedge(v::Vertex, w::Vertex) = push!(v.outneighbours, hash(w))


function collsetunion!(v::Vertex, collisions::Set{UInt64}) 
    v.collset = union(v.collset, collisions)
end


backsetappend!(w::Vertex, v::Vertex) = push!(w.backset, hash(v))


backset(G::ConfigGraph, v::Vertex) = map(w -> G[w], collect(v.backset))


getparent(G::ConfigGraph, v::Vertex) = isnothing(v.parent) ? nothing : G[v.parent]


outneighbours(G::ConfigGraph, v::Vertex) = map(w -> G[w], collect(v.outneighbours))


function setparent!(v::Vertex, parent::Vertex)
    v.parent = hash(parent)
end


hasvertex(G::ConfigGraph, config::Config) = haskey(G, hash(config))


getvertex(G::ConfigGraph, config::Config) = G[hash(config)]


end