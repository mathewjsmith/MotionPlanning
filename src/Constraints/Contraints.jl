module Constraints

using MotionPlanning.Model

using Primes
using SparseArrayKit

export
    Constraint,
    ClashConstraint,
    OverlapConstraint,
    ConstraintMatrix,
    isconstrained,
    constraintstomatrix


"""
 Enforces that `robot` cannot move onto `pos` at `time`.
"""
struct ClashConstraint
    time  :: Int
    pos   :: Pos
    robot :: Int
end

Base.:(==)(a::ClashConstraint, b::ClashConstraint) = a.time == b.time && a.pos == b.pos


"""
Enforces that `robot` cannot move onto `dst` *from* `src` at `time`.
"""
struct OverlapConstraint
    time  :: Int
    src   :: Pos
    dst   :: Pos
    robot :: Int
end

Base.:(==)(a::OverlapConstraint, b::OverlapConstraint) = a.time == b.time && a.src == b.src && a.dst == b.dst


const Constraint = Union{ClashConstraint, OverlapConstraint}


"""
A 4d matrix representing the space of possible constraints for an instance.

`constmat[r, x, y, t]` holds an integer `c` encoding which constraints are active for robot `r` at position `(x, y)` and time `t`.
- If `c == 0`, no constraints are active.
- If `c % 2 == 0`, the corresponding clash constraint is active.
- If `c % 3 == 0`, `r` cannot move *north* onto `(x, y)` at time `t`.
- If `c % 5 == 0`, `r` cannot move *south* onto `(x, y)` at time `t`.
- If `c % 7 == 0`, `r` cannot move *east* onto `(x, y)` at time `t`.
- If `c % 11 == 0`, `r` cannot move *west* onto `(x, y)` at time `t`.
- If `c % 13 == 0`, `r` cannot move *remain* on `(x, y)` at time `t`.

This prime number encoding is used to enable the `ConstraintMatrix` to be used as the encoding for the chromosome in a genetic algorithm.
"""
const ConstraintMatrix = SparseArray{Int16, 4}


"""
    constraintstomatrix(constraints, dims)

Convert a list of `constraints` to a constraint matrix.
"""
function constraintstomatrix(constraints::Vector{Constraint}, dims::NTuple{4, Int})
    mat = SparseArray{Int16, 4}(zeros(dims))

    for constraint in constraints
        t = constraint.time
        r = constraint.robot

        if isa(constraint, ClashConstraint)
            x, y = constraint.pos .+ 1
            v    = mat[r, x, y, t]

            mat[r, x, y, t] = v == 0 ? 2 : v * 2
        elseif isa(constraint, OverlapConstraint)
            x, y = constraint.dst .+ 1
            dir  = dirvalue(direction(constraint.src, constraint.dst))
            v    = mat[r, x, y, t]

            mat[r, x, y, t] = v == 0 ? dir : v * dir
        end
    end

    mat
end


"""
    isconstraint(r, src, dst, t, mat)

Determine if the constraint corresponding to `(r, src, dst, t)` is constrained.
"""
function isconstrained(r::Int, src::Pos, dst::Pos, t::Int, mat::ConstraintMatrix)
    src = src .+ 1
    dst = dst .+ 1

    v = mat[r, dst..., t]

    if v <= 0
        return false
    elseif v % 2 == 0
        return true
    end

    dirval = dirvalue(direction(src, dst))

    v % dirval == 0
end


"""
    dirvalue(dir)

Convert a direction encoded as a symbol (e.g. `:north`) into its corresponding prime number encoding.
"""
function dirvalue(dir::Symbol)
    if dir == :north
        3
    elseif dir == :south
        5
    elseif dir == :east
        7
    elseif dir == :west
        11
    elseif dir == :remain
        13
    else
        throw("not a valid direction.")
    end
end


"""
    direction(src, dst)

Determine the direction of a move from `src` to `dst`.
"""
function direction(src::Pos, dst::Pos)
    a, b = src
    c, d = dst

    if a - c < 0
        :east
    elseif a - c > 0
        :west
    elseif b - d > 0
        :north
    elseif b - d < 0
        :south
    else
        :remain
    end
end


end