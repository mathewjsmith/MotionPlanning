module Heuristics

using MotionPlanning.Model
using MotionPlanning.SingleRobotPlanning
using MotionPlanning.MultiRobotPlanning.Objectives

export
    Heuristic,
    MaxDist,
    SIC,
    ParetoMaxDistSIC,
    heuristic


abstract type Heuristic end


struct MaxDist <: Heuristic end

heuristic(::MaxDist, config::Config, target_config::Config)::Cost = [ maxdist(config, target_config) ]

weight(::MaxDist, ::Config, ::Config)::Cost = [ 1.0 ]

maxdist(config::Config, target_config::Config) = maximum(manhattandist(pos, target) for (pos, target) in zip(config, target_config))


struct SIC <: Heuristic end

heuristic(::SIC, config::Config, target::Config)::Cost = [ sumofindividualcosts(config, target) ]

sumofindividualcosts(config::Config, target::Config) = sum(manhattandist(pos, target) for (pos, target) in zip(config, target))


struct ParetoMaxDistSIC <: Heuristic end

heuristic(::ParetoMaxDistSIC, config::Config, target_config::Config)::Cost = [ heuristic(MaxDist(), config, target_config); heuristic(SIC(), config, target_config) ]

weight(::ParetoMaxDistSIC, curr_config::Config, next_config::Config)::Cost = [ weight(MaxDist(), curr_config, next_config); weight(SIC(), curr_config, next_config) ]


end