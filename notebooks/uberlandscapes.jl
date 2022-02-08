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

# ╔═╡ ffffb000-6da3-4161-b822-681b4686940b
function f(sol, inst, part)
    if isnothing(sol)
        0.0
    else
        colls  = length(findcollisions(sol, inst))
		k      = maximum(map(length, part))
        sig(x) = exp(-x) / (1 + exp(-x))
		
        sig(colls) * (1 + sig(k))
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
inst = generate(5, (5, 5), 0.0, "uberlandscape")

# ╔═╡ 525960ab-3e7a-4c03-a791-3e9b7b7405de
drawinst(inst)

# ╔═╡ 561c37a7-4d42-44d8-aa13-5d02c64b556a
dims = length(inst.robots), inst.dims..., 4 * sum(inst.dims)

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

	map(p -> f(p[1], inst, p[2]), 
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
begin
	gr()
	theme(:dao)
	
	xs = 1:length(interpolated[1])
	ys = 1:n-1
	g(x, y) = 2 * interpolated[y][x]
	
	plt = wireframe(
		xs, ys, g,
		xticks=(1:20:100, string.([])),
		zlims=(0, 1),
		axis=false,
		ylabel="coupling", zlabel="fitness"
	)

	savefig(plt, "uberlandscape2.pdf")

	plt
end

# ╔═╡ 71b2110d-5872-4806-bebf-f485893923fc


# ╔═╡ Cell order:
# ╠═eff6f29c-7dd2-11ec-02ee-bbf43ebda63a
# ╠═30ecd81d-5f28-4733-bd12-4d21183e1a40
# ╠═1bd42eb1-587f-4a29-90f9-be3e4f2c5480
# ╠═43c6276a-5393-4775-a44e-df255bce4515
# ╠═525960ab-3e7a-4c03-a791-3e9b7b7405de
# ╠═561c37a7-4d42-44d8-aa13-5d02c64b556a
# ╠═6d5a5a85-a3f8-42da-a5cc-b29e7946262a
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
