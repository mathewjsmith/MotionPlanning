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


struct ClashConstraint
    time  :: Int
    pos   :: Pos
    robot :: Int
end

Base.:(==)(a::ClashConstraint, b::ClashConstraint) = a.time == b.time && a.pos == b.pos


struct OverlapConstraint
    time  :: Int
    src   :: Pos
    dst   :: Pos
    robot :: Int
end

Base.:(==)(a::OverlapConstraint, b::OverlapConstraint) = a.time == b.time && a.src == b.src && a.dst == b.dst


const Constraint = Union{ClashConstraint, OverlapConstraint}


const ConstraintMatrix = SparseArray{Int16, 4}


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


# function getconstraints(mat::ConstraintMatrix, r, x, y, t)
#     constraintsfromint(r, x, y, t, mat[r, x, y, t])
# end


# constraintsfromint(r, x, y, t, i) = filter(!isnothing, [
#     i % 2  == 0 ? ClashConstraint(t, (x, y), r) : nothing,
#     i % 3  == 0 ? OverlapConstraint(t, getadjacent(v, :north),  v, r) : nothing,
#     i % 5  == 0 ? OverlapConstraint(t, getadjacent(v, :south),  v, r) : nothing,
#     i % 7  == 0 ? OverlapConstraint(t, getadjacent(v, :east),   v, r) : nothing,
#     i % 11 == 0 ? OverlapConstraint(t, getadjacent(v, :west),   v, r) : nothing,
#     i % 13 == 0 ? OverlapConstraint(t, getadjacent(v, :remain), v, r) : nothing
# ])


# function getadjacent(pos::Pos, dir::Symbol)
#     x, y = pos

#     if dir == :north
#         (x, y - 1)
#     elseif dir == :south
#         (x, y + 1)
#     elseif dir == :east
#         (x + 1, y)
#     elseif dir == :west
#         (x - 1, y)
#     elseif dir == :remain
#         (x, y)
#     else
#         nothing
#     end
# end


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
Returns the direction of a move from `src` to `dst`.
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