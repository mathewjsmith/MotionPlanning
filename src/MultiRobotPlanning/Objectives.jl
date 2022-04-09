module Objectives

using MotionPlanning.Model
using MotionPlanning.SingleRobotPlanning

export
    Cost,
    Objective,
    MakeSpan,
    TotalDist,
    weight


const Cost = Vector{Float64}


"""
    Base type for objective functions to be used with M*.
"""
abstract type Objective end


"""
    The MakeSpan objective for MRMP: the optimal plan is the one with the shortest maximum path for anyone robot. Edges in the configuration-space have unit weight.
"""
struct MakeSpan <: Objective end

weight(::MakeSpan, ::Config, ::Config)::Cost = [ 1.0 ]


"""
    The TotalDist objective for MRMP: the optimal plan is the one with the smallest sum of path lengths for all robots. Edges in the configuration-space are weighted by the number of robots that change position between the two configs.
"""
struct TotalDist <: Objective end

weight(::TotalDist, curr_config::Config, next_config::Config)::Cost = [
    sum(curr != next for (curr, next) in zip(curr_config, next_config))
]


end