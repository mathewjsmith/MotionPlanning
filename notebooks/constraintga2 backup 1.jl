### A Pluto.jl notebook ###
# v0.17.5

using Markdown
using InteractiveUtils

# ╔═╡ 22b8f8e0-72f4-11ec-0c12-3f9362497f6c
begin
	using Pkg
	Pkg.activate(joinpath(@__DIR__, ".."))
end

# ╔═╡ fa37fa7b-4dcd-4c2c-b76c-659e0828ec24
begin
	using Revise

	using MotionPlanning.Collisions
	using MotionPlanning.Model
	using MotionPlanning.IO
	using MotionPlanning.MultiRobotPlanning.Metrics
	using MotionPlanning.SingleRobotPlanning

	using Glob
end

# ╔═╡ 20769858-4b64-4944-b66d-3f3678862994
using MotionPlanning.GeneticAlgorithms.ConstraintGA2

# ╔═╡ b169a4c0-291f-45dc-b84a-a5031180df21
inst = readinstance.(glob("../instances/new/rand*n032*d16x16*c000*"))[1]

# ╔═╡ 4acb00e7-2ee7-436b-b1e7-4605d444f4b7
inst.dims = (16, 16)

# ╔═╡ 0503ba1c-10db-4f17-b52c-aefaa66ec9a1
initsol = plantosolution([ astar(r.pos, r.target, inst) for r in inst.robots ])

# ╔═╡ c06777a5-7f13-4c9e-bd69-aade196ef222
makespan(initsol, :solution)

# ╔═╡ 6a44d2b4-c917-45c7-a780-13ae17c93c90
findcollisions(initsol, inst)

# ╔═╡ 6f67a045-c25e-43f6-b117-aa63b17cf485
length(findcollisions(initsol, inst))

# ╔═╡ a5279a84-aeb2-4699-a1ca-cfc8aeac3f44
sol = evolve(inst, Params(32, 4, 0.95, 0.01, 30); maxgens=1024)[1]

# ╔═╡ d3e57d3c-2803-495d-8a4b-8cb70df94c0b
makespan(sol, :solution)

# ╔═╡ 991a5777-fa55-4e05-b590-82e3f65f4b6e
length(findcollisions(sol, inst))

# ╔═╡ addb86f3-aac3-4b9e-acfa-1cc52035840f
findcollisions(sol, inst)

# ╔═╡ 8972929f-c51d-494d-8905-455ac6e92fe4


# ╔═╡ 702035c0-050e-4f07-9a9d-1cf79343cfca


# ╔═╡ 54f9e4cd-e330-4fee-a58d-5c320ea55d2a


# ╔═╡ 268fc9e3-4648-4817-889a-ab1ec6745193


# ╔═╡ e9909ba9-4276-4fc6-8850-747b2405616d


# ╔═╡ 9a7b3df2-ebe0-4ddf-8587-b781f14b8653


# ╔═╡ c76b31ce-e998-43be-b224-fc60610f32d6


# ╔═╡ d878f650-0271-41ea-b837-3d0f2ecb64a1


# ╔═╡ Cell order:
# ╠═c1dd0490-b020-4469-baee-c62aa2e91800
# ╠═22b8f8e0-72f4-11ec-0c12-3f9362497f6c
# ╠═20769858-4b64-4944-b66d-3f3678862994
# ╠═fa37fa7b-4dcd-4c2c-b76c-659e0828ec24
# ╠═b169a4c0-291f-45dc-b84a-a5031180df21
# ╠═4acb00e7-2ee7-436b-b1e7-4605d444f4b7
# ╠═0503ba1c-10db-4f17-b52c-aefaa66ec9a1
# ╠═c06777a5-7f13-4c9e-bd69-aade196ef222
# ╠═6a44d2b4-c917-45c7-a780-13ae17c93c90
# ╠═6f67a045-c25e-43f6-b117-aa63b17cf485
# ╠═a5279a84-aeb2-4699-a1ca-cfc8aeac3f44
# ╠═d3e57d3c-2803-495d-8a4b-8cb70df94c0b
# ╠═991a5777-fa55-4e05-b590-82e3f65f4b6e
# ╠═addb86f3-aac3-4b9e-acfa-1cc52035840f
# ╠═8972929f-c51d-494d-8905-455ac6e92fe4
# ╠═702035c0-050e-4f07-9a9d-1cf79343cfca
# ╠═54f9e4cd-e330-4fee-a58d-5c320ea55d2a
# ╠═268fc9e3-4648-4817-889a-ab1ec6745193
# ╠═e9909ba9-4276-4fc6-8850-747b2405616d
# ╠═9a7b3df2-ebe0-4ddf-8587-b781f14b8653
# ╠═c76b31ce-e998-43be-b224-fc60610f32d6
# ╠═d878f650-0271-41ea-b837-3d0f2ecb64a1
