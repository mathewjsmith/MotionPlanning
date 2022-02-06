### A Pluto.jl notebook ###
# v0.17.5

using Markdown
using InteractiveUtils

# ╔═╡ 91214702-8770-11ec-03dd-f390f5455c3f
begin
	using Pkg
	Pkg.activate(joinpath(@__DIR__, ".."))
end

# ╔═╡ 2299a927-b51e-49f9-8166-2918f7bc380d
begin
	using MotionPlanning.Model
	using MotionPlanning.IO
	using MotionPlanning.Benchmarks
	using MotionPlanning.MultiRobotPlanning.CBS
end

# ╔═╡ c8db5dc5-7ccf-407f-ba78-604fe45524ea
inst = readinstance("../instances/new/rand_240_n004_d8x8_c000.instance.json")

# ╔═╡ a2f17052-51ae-4b92-b730-2f580c7888a4
bm = readbenchmark("../benchmarks/results/rand_240_n004_d8x8_c000_ConstraintGA")

# ╔═╡ 6bdbbdfd-eec7-4fb6-baa5-4602bec585a1
initplan = begin
	sol = bm.solution
	
	n = length(sol[1])
	l = length(sol)
	
	plan = Vector{Tuple{Int, Int}}[]
	
	for r in 1:n
		path = []
		for t in 1:l
			push!(path, sol[t][r])
		end

		push!(plan, path)
	end

	plan
end

# ╔═╡ ef976392-e8fd-435b-8e76-94a792840f76
gacbssol = cbs(inst, initplan)

# ╔═╡ 86bd3887-1575-49df-b8b8-2b1a00cb6352


# ╔═╡ Cell order:
# ╠═91214702-8770-11ec-03dd-f390f5455c3f
# ╠═2299a927-b51e-49f9-8166-2918f7bc380d
# ╠═c8db5dc5-7ccf-407f-ba78-604fe45524ea
# ╠═a2f17052-51ae-4b92-b730-2f580c7888a4
# ╠═6bdbbdfd-eec7-4fb6-baa5-4602bec585a1
# ╠═ef976392-e8fd-435b-8e76-94a792840f76
# ╠═86bd3887-1575-49df-b8b8-2b1a00cb6352
