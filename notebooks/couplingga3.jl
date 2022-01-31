### A Pluto.jl notebook ###
# v0.17.5

using Markdown
using InteractiveUtils

# ╔═╡ c62c2774-77fc-11ec-1642-572eb7e0fe00
begin
	using Pkg
	Pkg.activate(joinpath(@__DIR__, ".."))
end

# ╔═╡ 32a93ff9-e720-48f7-a5f2-83f8e3c284de
begin
	using Revise

	using MotionPlanning.Collisions
	using MotionPlanning.Model
	using MotionPlanning.IO
	using MotionPlanning.GeneticAlgorithms.CouplingGA3
	using MotionPlanning.MultiRobotPlanning.Metrics
	using MotionPlanning.MultiRobotPlanning.CBS
	using MotionPlanning.MultiRobotPlanning.MStar
	using MotionPlanning.SingleRobotPlanning

	using Glob
end

# ╔═╡ ab8ecb1a-109f-498b-873f-2ea9d017e070
inst = readinstance.(glob("../instances/new/rand*n016*"))[16]

# ╔═╡ f6a50f5b-2625-4ff8-b931-e34162496e77
inst.bounded = false

# ╔═╡ 04580d29-1fad-4905-a583-06d6ca003f79
initsol = plantosolution([ astar(r.pos, r.target, inst) for r in inst.robots ])

# ╔═╡ 6e98a813-e4fb-47da-a479-b699d2654baa
makespan(initsol, :solution)

# ╔═╡ db44cea6-d638-478c-9c7b-6cefb25eef62
length(findcollisions(initsol, inst))

# ╔═╡ 6219c0df-26b9-4667-8586-1217d02fb7fa
sol = evolve(inst, Params(32, 4, 0.85, 0.05, 5))

# ╔═╡ 896fe78e-52f1-4f46-8a9d-3493a402d2a2
makespan(sol, :solution)

# ╔═╡ b4c793ab-ecb0-4105-bd32-ec72024c2ed3
length(findcollisions(sol, inst))

# ╔═╡ e7268756-e331-43a1-825b-c98cb7b4fada
# cbs(inst)

# ╔═╡ Cell order:
# ╠═c62c2774-77fc-11ec-1642-572eb7e0fe00
# ╠═32a93ff9-e720-48f7-a5f2-83f8e3c284de
# ╠═ab8ecb1a-109f-498b-873f-2ea9d017e070
# ╠═f6a50f5b-2625-4ff8-b931-e34162496e77
# ╠═04580d29-1fad-4905-a583-06d6ca003f79
# ╠═6e98a813-e4fb-47da-a479-b699d2654baa
# ╠═db44cea6-d638-478c-9c7b-6cefb25eef62
# ╠═6219c0df-26b9-4667-8586-1217d02fb7fa
# ╠═896fe78e-52f1-4f46-8a9d-3493a402d2a2
# ╠═b4c793ab-ecb0-4105-bd32-ec72024c2ed3
# ╠═e7268756-e331-43a1-825b-c98cb7b4fada
