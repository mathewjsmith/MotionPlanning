Accompanying code for the master's thesis: [Coupled and Decoupled Approaches to Multi-Robot Motion Planning](thesis.pdf).

# Abstract
This thesis explores the role of coupling in the Multi-Robot Motion Planning problem (MRMP). Approaches to MRMP can be broadly categorised as either *coupled* or *decoupled*. It is widely taken to be the case that decoupled approaches, while much more efficient than their coupled counterparts, cannot be complete and optimal. This has however remained a matter of empirical observation. In this work, an algorithm-independent method of measuring the *degree of coupling* of an algorithm is developed. This in turn makes it possible to prove that (almost) complete coupling is necessary to guarantee completeness and optimality in general. Through the way coupling is defined, it follows that the size of the search-space grows exponentially with coupling, and thus the expected trade-off between performance and optimality is made rigorous. *Dynamically coupled* algorithms emerge from this as a compelling candidate to manage this trade-off, and their efficacy is verified through experiment. Dynamic coupling also holds a distinct analogy with the phenomenon of *gradual complexification* exhibited in naturally occurring complex systems. This motivates treating MRMP as an evolving complex system where solutions with the fewest collisions, arrived at with the least coupling, are the fittest, and a genetic algorithm for MRMP is designed based on this concept.

# Multi-Robot Motion Planning
Multi-Robot Motion Planning (MRMP) is the problem of, given a set of robots distributed in space, each with a target position, finding a plan which takes each robot from its starting position to its target position in the least number of moves *without collisions*. The constraint of collision avoidance is central to the complexity of the problem: without this constraint, it would be a simple matter of independently finding the shortest paths for each robot---a problem for which polynomial-time algorithms have long been known. Adding the deceptively simple constraint of collision avoidance launches us into the territory of NP-hardness.

# Algorithms
The code in this repository provides Julia implementations of the following algorithms for MRMP, along with infrastructure for benchmarking.
- M*: [Wagner and Choset 2015](https://www.sciencedirect.com/science/article/pii/S0004370214001271)
- Conflict Based Search (CBS): [Sharon et al. 2015](https://www.sciencedirect.com/science/article/pii/S0004370214001386)
- Priority Planning: the heuristic used is based similar to those used by multiple of the winning teams from the [2021 SoCG Challenge](https://arxiv.org/abs/2103.15381v1).
- ConstraintGA: this thesis.
