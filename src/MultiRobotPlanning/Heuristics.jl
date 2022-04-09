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


"""
Base type for heuristic functions to be used with M*.
"""
abstract type Heuristic end


"""
    The MaxDist heuristic for MRMP: characterised the maximum distance between anyone robot in `config` and its target in `target`.
"""
struct MaxDist <: Heuristic end

heuristic(::MaxDist, config::Config, target::Config)::Cost = [ maxdist(config, target) ]

maxdist(config::Config, target_config::Config) = maximum(
    manhattandist(pos, target) for (pos, target) in zip(config, target_config)
)


"""
    The Sum of Induced Costs (SIC) heuristic for MRMP: characterised the sum of the distances of each robot in `config` from their target in `target`.
"""
struct SIC <: Heuristic end

heuristic(::SIC, config::Config, target::Config)::Cost = [ sumofindividualcosts(config, target) ]

sumofindividualcosts(config::Config, target::Config) = sum(
    manhattandist(pos, target) for (pos, target) in zip(config, target)
)


"""
    Combination of the MaxDist and SIC heuristics. Characterised by a 2d vector of the form [ MaxDist, SIC ]. Ordering of two such values is achieved lexicographically.
"""
struct ParetoMaxDistSIC <: Heuristic end

heuristic(::ParetoMaxDistSIC, config::Config, target_config::Config)::Cost = [
    heuristic(MaxDist(), config, target_config); heuristic(SIC(), config, target_config)
]


end
