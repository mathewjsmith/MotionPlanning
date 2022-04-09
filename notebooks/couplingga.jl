### A Pluto.jl notebook ###
# v0.17.5

using Markdown
using InteractiveUtils

# ╔═╡ 63bd1d60-77b5-11ec-0ef6-995c806b0d05
begin
	using Pkg
	Pkg.activate(joinpath(@__DIR__, ".."))
end

# ╔═╡ 1bd1439e-a91f-41c7-a540-dd09c3fde1a5
begin
	using Revise

	using MotionPlanning.Collisions
	using MotionPlanning.Model
	using MotionPlanning.IO
	using MotionPlanning.GeneticAlgorithms.CouplingGA
	using MotionPlanning.MultiRobotPlanning.Metrics
	using MotionPlanning.MultiRobotPlanning.CBS
	using MotionPlanning.MultiRobotPlanning.MStar
	using MotionPlanning.MultiRobotPlanning.PriorityPlanning
	using MotionPlanning.SingleRobotPlanning
end

# ╔═╡ 4c0b25ba-b8c9-47df-b511-851195aadf25
inst = readinstance("../instances/random/random_11_0_5x5_5.instance.json")

# ╔═╡ 340e2003-c2f5-4a1e-8029-45266cb12c11
initsol = plantosolution([ ep(r.pos, r.target, inst) for r in inst.robots ])

# ╔═╡ 7fe8206b-9eeb-44a6-884d-70063280aa78
makespan(initsol, :solution)

# ╔═╡ 170c182a-ef6c-4222-86f2-89624af11b5d
length(findcollisions(initsol, inst))

# ╔═╡ 54f035c2-69a4-499f-8df8-0cf41db74f22
sol = evolve(inst, Params(32, 4, 0.95, 0.01, 5), initsol)

# ╔═╡ be58de42-1bdc-49bd-bc06-191d2e4fbeed
makespan(sol, :solution)

# ╔═╡ e1188faf-7c35-43b4-8c54-38ae21289ea7
length(findcollisions(sol, inst))

# ╔═╡ 56b54c3c-eff7-4a7d-b366-a56b0a7913ed
itersol = iterevolve(inst, Params(32, 4, 0.95, 0.01, 4))

# ╔═╡ 9c9cd250-8ce0-4628-89a7-faf7103e0da3
makespan(itersol, :solution)

# ╔═╡ 7e88f2b2-1c02-420c-87d2-fdcd9c8df671
length(findcollisions(itersol, inst))

# ╔═╡ 8f7c3311-b881-4fc6-aa0d-62cf80fb3de3
shadsol = priorityplanning(inst)

# ╔═╡ 93b20215-640c-43ef-a4f0-e4dd715124a4
makespan(shadsol, :solution)

# ╔═╡ 90e9ea6c-e3aa-44fa-8a39-f5d10ff5f417
length(findcollisions(shadsol, inst))

# ╔═╡ Cell order:
# ╠═63bd1d60-77b5-11ec-0ef6-995c806b0d05
# ╠═1bd1439e-a91f-41c7-a540-dd09c3fde1a5
# ╠═4c0b25ba-b8c9-47df-b511-851195aadf25
# ╠═340e2003-c2f5-4a1e-8029-45266cb12c11
# ╠═7fe8206b-9eeb-44a6-884d-70063280aa78
# ╠═170c182a-ef6c-4222-86f2-89624af11b5d
# ╠═54f035c2-69a4-499f-8df8-0cf41db74f22
# ╠═be58de42-1bdc-49bd-bc06-191d2e4fbeed
# ╠═e1188faf-7c35-43b4-8c54-38ae21289ea7
# ╠═56b54c3c-eff7-4a7d-b366-a56b0a7913ed
# ╠═9c9cd250-8ce0-4628-89a7-faf7103e0da3
# ╠═7e88f2b2-1c02-420c-87d2-fdcd9c8df671
# ╠═8f7c3311-b881-4fc6-aa0d-62cf80fb3de3
# ╠═93b20215-640c-43ef-a4f0-e4dd715124a4
# ╠═90e9ea6c-e3aa-44fa-8a39-f5d10ff5f417
