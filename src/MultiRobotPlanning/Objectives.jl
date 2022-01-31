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


abstract type Objective end


struct MakeSpan <: Objective end

weight(::MakeSpan, ::Config, ::Config)::Cost = [ 1.0 ]


struct TotalDist <: Objective end

weight(::TotalDist, curr_config::Config, next_config::Config)::Cost = [ sum(curr != next for (curr, next) in zip(curr_config, next_config)) ]


end