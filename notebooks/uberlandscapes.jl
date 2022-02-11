### A Pluto.jl notebook ###
# v0.17.5

using Markdown
using InteractiveUtils

# ╔═╡ eff6f29c-7dd2-11ec-02ee-bbf43ebda63a
begin
	using Pkg
	Pkg.activate(joinpath(@__DIR__, ".."))
end

# ╔═╡ 30ecd81d-5f28-4733-bd12-4d21183e1a40
begin
	using Revise

	using MotionPlanning.Collisions
	using MotionPlanning.Constraints
	using MotionPlanning.Model
	using MotionPlanning.MultiRobotPlanning.CBS
	using MotionPlanning.Benchmarks.Generators
	using MotionPlanning.SingleRobotPlanning

	using Images
	using OffsetArrays
	using Interpolations
	using Plots
	using PlotThemes
	using StaticArrays
	using Combinatorics
	using SparseArrayKit
	using LaTeXStrings
end

# ╔═╡ 1bd42eb1-587f-4a29-90f9-be3e4f2c5480


# ╔═╡ 43c6276a-5393-4775-a44e-df255bce4515
function drawinst(instance)
	w, h = instance.dims
	img = OffsetArray(repeat([RGB(1)], instance.dims...), 0:w-1, 0:h-1)
	for r in instance.robots
		img[r.pos...] = RGB(0, 1, 0)
	end
	for o in instance.obstacles
		img[o...] = RGB(0)
	end
	img
end

# ╔═╡ d3ce5ab4-92c8-44f3-b03a-2e994b0ac4e2
sig(x) = 1 / (1 + exp(-x))

# ╔═╡ 60c02ceb-0598-4ae8-8a13-1c890504e59b
relent(c, nn, mk, ww, hh) = let
	c = c == 0 ? 1 : c
	p = c / (nn * mk)
	q = min(0.99,12 * (nn - 1) / 5 * hh * ww)
	p * log2(p / q) + (1 - p) * log2((1 - p) / (1 - q))
end

# ╔═╡ ffffb000-6da3-4161-b822-681b4686940b
function f(sol, inst, part, nn, ww, hh)
    if isnothing(sol)
        0.0
    else
        colls  = length(findcollisions(sol, inst))
		k      = maximum(map(length, part))
		mk     = length(sol)
        relent(colls, nn, mk, ww, hh) #* sig((nn - k) / nn)
		# sig(colls) * sig(k)
    end
end

# ╔═╡ e1c9e62e-0e17-48c4-b4bc-2f2596103418
function mergesolutions(left::Solution, right::Solution, group::Vector{Int})
    if length(left) < length(right)
        for _ in length(left) + 1 : length(right)
            push!(left, left[end])
        end
    end

    if length(right) < length(left)
        for _ in length(right) + 1 : length(left)
            push!(right, right[end])
        end
    end

    for (time, config) in enumerate(left)
        for (i, r) in enumerate(group)
            config[r] = right[time][i]
        end
    end

    left
end

# ╔═╡ 6a2ff430-9976-4656-8f2f-a1520ee497bf
function evenlyspaced(a, b, n)
    h = (b-a)/(n-1)
    collect(a:h:b)
end

# ╔═╡ f5a84d35-185c-459a-b592-82d14bd23f84
inst = generate(5, (4, 4), 0.0, "uberlandscape")

# ╔═╡ 525960ab-3e7a-4c03-a791-3e9b7b7405de
drawinst(inst)

# ╔═╡ 561c37a7-4d42-44d8-aa13-5d02c64b556a
dims = length(inst.robots), inst.dims..., 2 * sum(inst.dims)

# ╔═╡ 6d5a5a85-a3f8-42da-a5cc-b29e7946262a
n, w, h, d = dims

# ╔═╡ aca14581-7bea-4c93-afa5-127221ac6789
parts = collect(partitions(1:n))

# ╔═╡ f1bc6432-cf89-4ded-8676-affd1c700246
fitnesses = begin
	solutions = []
	
	zerosol = plantosolution([ astar(r.pos, r.target, inst) for r in inst.robots])
	
	for part in parts
		partsol = deepcopy(zerosol)
				
		for group in part
			groupinst = deepcopy(inst)
			groupinst.robots = filter(r -> r.id ∈ group, groupinst.robots)
			result = cbs(groupinst)

			if isnothing(result)
				partsol = nothing
				break
			end
				
			groupsol = result[1]

			partsol = mergesolutions(partsol, groupsol, group)
		end
		
		push!(solutions, partsol)
	end

	map(p -> f(p[1], inst, p[2], n, w, h), 
		zip(solutions, parts)
	)
end

# ╔═╡ ccc02472-c9b6-4351-a382-e53e57fb3842
groups = begin
	groups = [ [] for _ in 1:n ]

	for (part, fit) in zip(parts, fitnesses)
		k = maximum(map(length, part))
		push!(groups[k], fit)
	end

	groups
end

# ╔═╡ eacd9d0a-061a-4d54-ad7b-de5fe81b4dac
interpolators = map(
	x -> CubicSplineInterpolation(1:length(x), SVector{length(x), Float64}(x)), 
	groups[2:end-1]
)

# ╔═╡ 3731aa90-1d49-4364-9ccb-e9154b686ac2
interpolated = [
	if 1 < k < n
		[ interpolators[k-1](x) for x in evenlyspaced(1, length(groups[k]), 100) ]
	else
		repeat(groups[k], 100)
	end
	for k in 1:n
]

# ╔═╡ 4a00ec10-48be-4ad4-9b8c-f1d9af066d9d
let
	gr()
	theme(:dao)
	
	xs = 1:length(interpolated[1])
	ys = 1:n
	g(x, y) = 2 * interpolated[y][x]
	
	plt = contour(
		xs, ys, g,
		xticks=(1:20:100, string.([])),
		c = cgrad(:matter, rev=true),
		fill = true,
		axis=false,
		ylabel=L"\kappa",
		colorbar=false,
		colorbar_title= L"f_I(P)",
		colorbar_tickfontcolor = :white,
		colorbar_titlefontrotation = -90,
		title = string(L"(a) f_I(P)", "  (relative entropy only)")
	)

	savefig(plt, "contourlandscape-no_k.pdf")

	plt
end

# ╔═╡ 71b2110d-5872-4806-bebf-f485893923fc
let
	gr()
	theme(:dao)
	
	xs = 1:length(interpolated[1])
	ys = 1:n-1
	g(x, y) = y == 1 ? 2 * interpolated[y][x] : 0.0

	plt = plot(
		xs, repeat([1], length(xs)), interpolated[1][:],
		xticks=(1:20:100, string.([])),
		zlims=(0, 1),
		axis=false,
		ylabel="coupling", zlabel="fitness",
		c = :thermal
	)

	for i in 2:n-1
		plot!(
			xs, repeat([i], length(xs)), interpolated[i][:],
			xticks=(1:20:100, string.([])),
			zlims=(0, 1),
			axis=false,
			ylabel="coupling", zlabel="fitness"
		)
	end

	plt
end

# ╔═╡ 5a86aea2-b59b-48e7-b121-e537d231b8c5
constraints = let

	rs = 1:n
	xs = 1:w
	ys = 1:h
	ts = 1:d

	cs = [2, 3, 5, 7, 11, 13]

	reduce(vcat, [
		(r, x, y, t, c)
		for r in rs, x in xs, y in ys, t in ts, c in cs
	])
end

# ╔═╡ 544ecc3a-5feb-4ec7-bcea-7ebf5c68db3c
chroms = let 	
	combs = [ combinations(constraints, k) for k in 1:20 ]

	chroms = [
		(
			begin
				chrom = SparseArray{UInt16, 4}(zeros(n, w, h, d))
				for (r, x, y, t, c) in comb
					v = chrom[r, x, y, t]
					chrom[r, x, y, t] = v == 0 ? c : c * v
				end
				chrom
			end
			for comb in combs[k]
		) for k in 1:20
	]
end		

# ╔═╡ 459040bf-30d3-4b23-b871-306a66696dff
# collect(chroms[2])

# ╔═╡ b9829f58-d672-4108-b347-3bcc159ec47e
const Chromosome = ConstraintMatrix

# ╔═╡ 4bf60b05-f631-46d1-9c48-3e4e2966a437
genes(r::Int, chrom::Chromosome) = chrom[r, :, :, :]

# ╔═╡ c0815fab-a00b-41f6-861b-df8af97f4203
function getpaths(paths::Dict{UInt64, Union{Path, Nothing}}, chrom::Chromosome, inst::MRMPInstance)
    n = size(chrom)[1]

    chrompaths = Vector{Tuple{UInt64, Bool, Path}}()

    chrompaths = for r in 1:n
        g = genes(r, chrom)
        h = hash((r, g))

        if !haskey(paths, h)
            robot = inst.robots[r]
            path = astar(robot.pos, robot.target, inst; constmat=chrom, r=r)

            (h, true, path)
        else
            path = paths[h]

            (h, false, path)
        end
    end

    chrompaths
end

# ╔═╡ 27839d4f-309a-4cc4-8b79-c00d2024f8bd
function solve!(paths::Dict{UInt64, Union{Path, Nothing}}, chrom::Chromosome, inst::MRMPInstance)
    chrompaths = getpaths(paths, chrom, inst)
    plan       = map(last, chrompaths)
    newpaths   = map(t -> (t[2], t[3]), filter(p -> p[2], chrompaths))

    for (gh, path) in newpaths
        paths[gh] = path
    end
    # updatepaths!(paths, chrom, inst)

    # plan = [ paths[hash((r, genes(r, chrom)))] for r in 1:size(chrom)[1] ]

    if any(map(isnothing, plan))
        return nothing
    end

    plantosolution(plan)
end

# ╔═╡ 6925300d-9492-4edf-8b83-b2083b3d04f0
function kfitnesses(kchroms::Vector{Chromosome})
	paths = Dict{UInt64, Union{Path, Nothing}}()

	fitnesses = Float64[]
	
	for chrom in kchroms
		sol = solve!(paths, chrom, inst)
		if isnothing(sol)
			push!(fitnesses, 0.0)
		else
			colls  = length(findcollisions(sol, inst))
			push!(fitnesses, sig(colls))
		end
	end

	fitnesses
end

# ╔═╡ 117025b5-0b5f-4473-829b-9b45dccb1be7
let
	gr()
	theme(:dao)
	
	xs = 1:length(interpolated[1])
	ys = 1:n-1
	g(x, y) = y == 1 ? 2 * interpolated[y][x] : 0.0

	plt = plot(
		xs, repeat([1], length(xs)), interpolated[1][:],
		xticks=(1:20:100, string.([])),
		zlims=(0, 1),
		axis=false,
		ylabel="coupling", zlabel="fitness",
		c = :thermal
	)

	for i in 2:n-1
		plot!(
			xs, repeat([i], length(xs)), interpolated[i][:],
			xticks=(1:20:100, string.([])),
			zlims=(0, 1),
			axis=false,
			ylabel="coupling", zlabel="fitness"
		)
	end

	plt
end

# ╔═╡ d19df133-a4e0-4567-8e5b-a034bc5ac2fe
let
	ys = collect(1:10)
	xs = [ 1 : 10 * x for x in ys ]
	heatmap(1:20, 1:20, (x, y) -> rand())
end

# ╔═╡ Cell order:
# ╠═eff6f29c-7dd2-11ec-02ee-bbf43ebda63a
# ╠═30ecd81d-5f28-4733-bd12-4d21183e1a40
# ╠═1bd42eb1-587f-4a29-90f9-be3e4f2c5480
# ╠═43c6276a-5393-4775-a44e-df255bce4515
# ╠═525960ab-3e7a-4c03-a791-3e9b7b7405de
# ╠═561c37a7-4d42-44d8-aa13-5d02c64b556a
# ╠═6d5a5a85-a3f8-42da-a5cc-b29e7946262a
# ╠═d3ce5ab4-92c8-44f3-b03a-2e994b0ac4e2
# ╠═60c02ceb-0598-4ae8-8a13-1c890504e59b
# ╠═ffffb000-6da3-4161-b822-681b4686940b
# ╠═e1c9e62e-0e17-48c4-b4bc-2f2596103418
# ╠═aca14581-7bea-4c93-afa5-127221ac6789
# ╠═f1bc6432-cf89-4ded-8676-affd1c700246
# ╠═ccc02472-c9b6-4351-a382-e53e57fb3842
# ╠═eacd9d0a-061a-4d54-ad7b-de5fe81b4dac
# ╠═6a2ff430-9976-4656-8f2f-a1520ee497bf
# ╠═3731aa90-1d49-4364-9ccb-e9154b686ac2
# ╠═f5a84d35-185c-459a-b592-82d14bd23f84
# ╠═4a00ec10-48be-4ad4-9b8c-f1d9af066d9d
# ╠═71b2110d-5872-4806-bebf-f485893923fc
# ╠═5a86aea2-b59b-48e7-b121-e537d231b8c5
# ╠═544ecc3a-5feb-4ec7-bcea-7ebf5c68db3c
# ╠═459040bf-30d3-4b23-b871-306a66696dff
# ╠═b9829f58-d672-4108-b347-3bcc159ec47e
# ╠═4bf60b05-f631-46d1-9c48-3e4e2966a437
# ╠═c0815fab-a00b-41f6-861b-df8af97f4203
# ╠═27839d4f-309a-4cc4-8b79-c00d2024f8bd
# ╠═6925300d-9492-4edf-8b83-b2083b3d04f0
# ╠═117025b5-0b5f-4473-829b-9b45dccb1be7
# ╠═d19df133-a4e0-4567-8e5b-a034bc5ac2fe
