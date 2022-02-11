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

# ╔═╡ 391e0837-7f43-4a3c-b127-bf1ab1a5e607
begin
	using Plots
	using StatsPlots
	using Statistics
	using DataFrames
end

# ╔═╡ d74045e4-1a97-4844-b4c9-10c5ca5b465d
using LaTeXStrings

# ╔═╡ 45d2dc34-1384-4e6a-b96e-220113a42344
using DataInterpolations

# ╔═╡ b169a4c0-291f-45dc-b84a-a5031180df21
inst = readinstance.(glob("../instances/new/rand*n008*d8x8*c100*"))[1]

# ╔═╡ 4acb00e7-2ee7-436b-b1e7-4605d444f4b7
inst.dims = (8, 8)

# ╔═╡ 0503ba1c-10db-4f17-b52c-aefaa66ec9a1
initsol = plantosolution([ astar(r.pos, r.target, inst) for r in inst.robots ])

# ╔═╡ 6f67a045-c25e-43f6-b117-aa63b17cf485
length(findcollisions(initsol, inst))

# ╔═╡ b4f7cea9-2a99-4af6-ad8c-28d6a5a9092d
ngens = 512

# ╔═╡ a5279a84-aeb2-4699-a1ca-cfc8aeac3f44
sol, stats = evolve(inst, Params(32, 2, 0.95, 0.01, 30); maxgens=ngens)

# ╔═╡ bc2a9dc2-b04f-4127-a2aa-2b189d3f13c4
df = DataFrame(stats)

# ╔═╡ e93c8f8a-98a8-4666-a88f-4b5eb2ae5c2b
movingaverage(g, n) = [
	i < n ? mean(g[begin:i]) : mean(g[i-n+1:i]) 
	for i in 1:length(g)
]

# ╔═╡ d44fa124-3ae7-4d50-9ccb-7c22199ecb8f
movingmax(g, n) = [
	i < n ? maximum(g[begin:i]) : maximum(g[i-n+1:i]) 
	for i in 1:length(g)
]

# ╔═╡ 8972929f-c51d-494d-8905-455ac6e92fe4
gaprogplot = let
	theme(:dao)

	scale = mean(df.nconstraints) / mean(df.mincolls)
	
	@df df plot(1:ngens, movingaverage(df.nconstraints, 8) ./ scale,
		linewidth = 2,
		linecolor = RGB(222/255, 141/255, 195/255),
		label =  string(L"H_k", " (mean, scaled)")
	)
	
	@df df plot!(1:ngens, df.nconstraints ./ scale,
		linewidth = 1,
		linecolor = RGB(222/255, 141/255, 195/255),
		linealpha = 0.5,
		label = false
	)
	
	@df df plot!(1:ngens, :mincolls,
		linewidth = 2,
		linecolor = RGB(73/255, 169/255, 116/255),
		fillcolor = RGB(73/255, 169/255, 116/255),
		fill = true, fillalpha = 0.25,
		label = string(L"H_c", " (best)"),
		xlabel = "Generation", ylabel = string(L"H_c", " (best)"),
		xlims = (1, 512), ylims = (0, 10),
		xticks = 0:50:512, yticks = 0:1:10,
		legend = :topright,
		title = "ConstraintGA Progress"
	)

	improvements = filter(p -> !isnothing(p[2]), [ 
		begin
			if i == 1
				(i, c)
			else
				c == df.mincolls[i - 1] ? (i, nothing) : (i, c)
			end
		end
		for (i, c) in enumerate(df.mincolls)
	])

	impdf = DataFrame()
	impdf.gen = map(first, improvements)
	impdf.mincolls = map(last, improvements)

	@df impdf scatter!(:gen, :mincolls, 
		markers = (:diamond, 6), markerstrokecolor = [RGB(44/255, 162/255, 95/255)],
		label = false
	)
end

# ╔═╡ 9d2a91a2-cf31-4ff1-9591-20a631a1b12a
# savefig(gaprogplot, "gaprogplot-nokpenalty.pdf")

# ╔═╡ 54f9e4cd-e330-4fee-a58d-5c320ea55d2a


# ╔═╡ 268fc9e3-4648-4817-889a-ab1ec6745193


# ╔═╡ e9909ba9-4276-4fc6-8850-747b2405616d


# ╔═╡ 9a7b3df2-ebe0-4ddf-8587-b781f14b8653


# ╔═╡ c76b31ce-e998-43be-b224-fc60610f32d6


# ╔═╡ d878f650-0271-41ea-b837-3d0f2ecb64a1


# ╔═╡ Cell order:
# ╠═22b8f8e0-72f4-11ec-0c12-3f9362497f6c
# ╠═20769858-4b64-4944-b66d-3f3678862994
# ╠═fa37fa7b-4dcd-4c2c-b76c-659e0828ec24
# ╠═391e0837-7f43-4a3c-b127-bf1ab1a5e607
# ╠═b169a4c0-291f-45dc-b84a-a5031180df21
# ╠═4acb00e7-2ee7-436b-b1e7-4605d444f4b7
# ╠═0503ba1c-10db-4f17-b52c-aefaa66ec9a1
# ╠═6f67a045-c25e-43f6-b117-aa63b17cf485
# ╠═b4f7cea9-2a99-4af6-ad8c-28d6a5a9092d
# ╠═a5279a84-aeb2-4699-a1ca-cfc8aeac3f44
# ╠═bc2a9dc2-b04f-4127-a2aa-2b189d3f13c4
# ╠═d74045e4-1a97-4844-b4c9-10c5ca5b465d
# ╠═45d2dc34-1384-4e6a-b96e-220113a42344
# ╠═e93c8f8a-98a8-4666-a88f-4b5eb2ae5c2b
# ╠═d44fa124-3ae7-4d50-9ccb-7c22199ecb8f
# ╠═8972929f-c51d-494d-8905-455ac6e92fe4
# ╠═9d2a91a2-cf31-4ff1-9591-20a631a1b12a
# ╠═54f9e4cd-e330-4fee-a58d-5c320ea55d2a
# ╠═268fc9e3-4648-4817-889a-ab1ec6745193
# ╠═e9909ba9-4276-4fc6-8850-747b2405616d
# ╠═9a7b3df2-ebe0-4ddf-8587-b781f14b8653
# ╠═c76b31ce-e998-43be-b224-fc60610f32d6
# ╠═d878f650-0271-41ea-b837-3d0f2ecb64a1
