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


"""
A vertex corresponding to a config in the configuration-space, along with relevant data used by M*.
"""
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


"""
A graph representation of the (partial) configuration-space, used by M*.
"""
const ConfigGraph = Dict{UInt64, Vertex}

Base.string(G::ConfigGraph) = [ string(v) for v in values(G)]


"""
    addvertex!(G, config, cost, parent)

Add a vertex for `config` to `G`.
"""
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


"""
    nvertices(G)

Count the number of vertices in `G`.
"""
nvertices(G::ConfigGraph) = length(G)


"""
    addedge(v, w)

Add an edge from `v` to `w`.
"""
addedge(v::Vertex, w::Vertex) = push!(v.outneighbours, hash(w))


"""
    collsetunion!(v, collisions)

Add new collisions to the collset of `v`.
"""
function collsetunion!(v::Vertex, collisions::Set{UInt64}) 
    v.collset = union(v.collset, collisions)
end


"""
    backsetappend!(w, v)

Add `v` to the backset of `w`.
"""
backsetappend!(w::Vertex, v::Vertex) = push!(w.backset, hash(v))


"""
    backset(G, v)

Collect the vertices in the backset of `v`.
"""
backset(G::ConfigGraph, v::Vertex) = map(w -> G[w], collect(v.backset))


"""
    getparent(G, v)

Get the parent of `v`.
"""
getparent(G::ConfigGraph, v::Vertex) = isnothing(v.parent) ? nothing : G[v.parent]


"""
    outneighbours(G, v)

Collect the outgoing neighbours of `v`.
"""
outneighbours(G::ConfigGraph, v::Vertex) = map(w -> G[w], collect(v.outneighbours))


"""
    setparent!(v, parent)

Set the parent of `v` to be `parent`.
"""
function setparent!(v::Vertex, parent::Vertex)
    v.parent = hash(parent)
end


"""
    hasvertex(G, config)

Determine if a vertex corresponding to `config` already exists in `G`.
"""
hasvertex(G::ConfigGraph, config::Config) = haskey(G, hash(config))


"""
    getvertex(G, config)

Get the vertex in `G` corresponding to `config`.
"""
getvertex(G::ConfigGraph, config::Config) = G[hash(config)]


end