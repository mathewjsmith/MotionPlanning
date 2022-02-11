### A Pluto.jl notebook ###
# v0.17.5

using Markdown
using InteractiveUtils

# ╔═╡ 7b1ce480-880f-11ec-0efd-b7aaebe5995e
begin
	using Pkg
	Pkg.activate(joinpath(@__DIR__, ".."))
end

# ╔═╡ c3aef5cd-025f-435c-8bd2-0339dc41e6fa
begin
	using Revise

	using MotionPlanning.IO
	using MotionPlanning.Benchmarks
	using MotionPlanning.Model
	using MotionPlanning.MultiRobotPlanning.Metrics
	using MotionPlanning.SingleRobotPlanning
	using MotionPlanning.Collisions

	using DataFrames
	using DataFramesMeta
	using Plots
	using StatsPlots
	using Glob
	using StatsBase
	using LaTeXStrings
	using CategoricalArrays
end

# ╔═╡ f669f189-1236-4d98-bb06-42c1a6348822
getclustering(bm::Benchmark) = parse(Int, bm.instancename[end - 2 : end])

# ╔═╡ 84906ae8-c171-4767-8f1e-c122d6d32858
function replacenothing!(df)
	mapcols!(c -> replace(c, nothing => missing), df)
end

# ╔═╡ e9aa333d-8414-4df5-8084-e3bb1a7d904d
groupdf(df) = combine(
		groupby(df, :nrobots),
		:algorithm,
		:clustering,
		:time => mean ∘ skipmissing => :time,
		:avgkappa => mean ∘ skipmissing => :avgavg,
		:timedout => (ts -> sum([ !t for t in ts])) => :nsuccess
)

# ╔═╡ fe586b09-e023-4c88-b25a-5c75f35e4f59
theme(:dao)

# ╔═╡ 5661c146-9ad4-4ed0-aef5-ca6c5a200976
getclustering(inst::MRMPInstance) = parse(Int, inst.name[end - 2 : end]) / 100

# ╔═╡ 5ebead4c-59bc-41c9-917b-6294d3a480cf
function createdf(bms)
	df  = DataFrame(bms)
	
	df.clustering = map(getclustering, bms)

	df = transform(df, :time => ByRow(t -> t == 600.0) => :timedout)

	df = replacenothing!(df)

	df.ncolls = map(bm -> begin
		inst = try
			readinstance("../instances/new/$(bm.instancename).instance.json")
		catch _
			nothing
		end
		if isnothing(inst)
			missing
		else
			sol = plantosolution([ astar(r.pos, r.target, inst) for r in inst.robots ])
			length(findcollisions(sol, inst))
		end
	end, bms)
		
	select(df, [
		:algorithm, 
		:instancename, 
		:nrobots, 
		:time, 
		:makespan, 
		:totalmoves,
		:maxkappa,
		:avgkappa,
		:clustering,
		:timedout,
		:ncolls
	])
end

# ╔═╡ ba78572c-ed64-4905-bc88-3fab7fdf038b
mstardf = createdf(readbenchmark.(glob("../benchmarks/results/rand*8x8*MStar")))

# ╔═╡ 2395fd23-6a72-4c54-882f-dc14e5617346
mstargrouped = groupdf(mstardf)

# ╔═╡ 41336393-d3c3-483f-8120-be1860e64b8b
mstarcluster = combine(
	groupby(mstardf, [:nrobots, :clustering]),
	:time => mean => :time,
	:timedout => (ts -> 4 * sum([ !t for t in ts])) => :nsuccess,
	:avgkappa => mean ∘ skipmissing => :avgavg
)

# ╔═╡ 53fe147b-cc9e-4244-911c-21cb42a4c880
cbsdf = createdf(readbenchmark.(glob("../benchmarks/results/rand*8x8*_CBS")))

# ╔═╡ 771f3abf-f394-402b-82c5-f2bddd7f8e06
cbsgrouped = groupdf(cbsdf)

# ╔═╡ a6e24e73-dc86-4b40-ad42-924545fe29ac
avgkplot = let
	df = vcat(mstardf, cbsdf)

	@df df[completecases(df), :] groupedboxplot(
		:nrobots, 
		:avgkappa,
		group = :algorithm,
		fillalpha = 1.0, linewidth = 1,
		linecolor=[RGB(0/255, 109/255, 44/255) RGB(179/255, 0/255, 0/255)],
		ylims = [0, 12], yticks = 0:12,
		palette = :Set2_3,
		outliers = false,
		legend = :false,
		xlabel = string(L"n", " (robots)"),
		ylabel = string(L"\hat{\kappa}", " (average coupling)"),
		title = "(b) Average Degree of Coupling"
	)
end

# ╔═╡ a1aaec89-5345-42ae-b79d-ae0f79bd79ad
obvkplot = let
	df = vcat(mstardf, cbsdf)

	@df df[completecases(df), :] groupedboxplot(
		:nrobots, 
		:maxkappa,
		group = :algorithm,
		fillalpha=1.0, linewidth=1,
		linecolor=[RGB(0/255, 109/255, 44/255) RGB(179/255, 0/255, 0/255)],
		ylims = [0, 12], yticks = 0:12,
		palette = :Set2_3,
		outliers = false,
		legend = :topright,
		xlabel = string(L"n", " (robots)"),
		ylabel = string(L"\kappa", " (observed coupling)"),
		title = "(a) Observed Degree of Coupling",
	)
end

# ╔═╡ 63866a15-b48d-4855-85e2-3888567ec567
cbscluster = combine(
	groupby(cbsdf, [:nrobots, :clustering]),
	:time => mean => :time,
	:timedout => (ts -> 4 * sum([ !t for t in ts])) => :nsuccess,
	:avgkappa => mean ∘ skipmissing => :avgavg
)

# ╔═╡ 2824d793-1053-4b26-8c2a-37e305c7382e
clustkplot = let
	df = vcat(cbsgrouped, mstargrouped)

	@df df plot(:nrobots, :avgavg, group = :algorithm,
		linewidth = 2,
		palette = :Set2_3,
		marker = (:diamond, 6),
		markerstrokecolor=[RGB(44/255, 162/255, 95/255) RGB(227/255, 75/255, 51/255)],
		xticks=4:16,
		yticks=0:2:20,
		labels = ["CBS" "M*" "A*"], legend = :topright,
		xlabel = string(L"n", " (robots)"),
		ylabel = string(L"\hat{\kappa}", " (average coupling)"),
		title = "(a) Average degree of doupling w.r.t. clustering"
	)

	ctgcbs = CategoricalArray(cbscluster.clustering)
	levels!(ctgcbs, [66, 100, 0, 33])
	@df cbscluster scatter!(:nrobots, :avgavg, group = ctgcbs, 
		palette = :BuGn_4,
		labels = false,
		markerstrokewidth=1, markerstrokecolor=RGB(0/255, 109/255, 44/255)
	)
	
	ctgmstar = CategoricalArray(mstarcluster.clustering)
	levels!(ctgmstar, [66, 100, 0, 33])
	@df mstarcluster scatter!(:nrobots, :avgavg, group = ctgmstar,
		palette = :OrRd_4,
		labels = false,
		markerstrokewidth=1, markerstrokecolor=RGB(179/255, 0/255, 0/255)
	)
end

# ╔═╡ e82aba7d-8576-4721-8642-7639608ca255
clustsrateplot = let
	df = vcat(cbsgrouped, mstargrouped)

	ctg = CategoricalArray(df.algorithm)
	levels!(ctg, ["CBS", "MStar"])
	
	@df df plot(:nrobots, :nsuccess, group = ctg,
		linewidth = 2,
		palette = :Set2_3,
		marker = (:diamond, 6),
		markerstrokecolor=[RGB(44/255, 162/255, 95/255) RGB(227/255, 75/255, 51/255)],
		xticks=4:16, xlabel=string(L"n", " (robots)"),
		yticks=0:2:20, ylabel="Solved Instances",
		labels = ["CBS" "M*" "A*"], legend = false,
		title = "(b) Success rate w.r.t. clustering"
	)

	ctgcbs = CategoricalArray(cbscluster.clustering)
	levels!(ctgcbs, [66, 100, 0, 33])
	@df cbscluster scatter!(:nrobots, :nsuccess, group = ctgcbs, 
		palette = :BuGn_4,
		labels = false,
		markerstrokewidth=1, markerstrokecolor=RGB(0/255, 109/255, 44/255)
	)
	
	ctgmstar = CategoricalArray(mstarcluster.clustering)
	levels!(ctgmstar, [66, 100, 0, 33])
	@df mstarcluster scatter!(:nrobots, :nsuccess, group = ctgmstar,
		palette = :OrRd_4,
		labels = false,
		markerstrokewidth=1, markerstrokecolor=RGB(179/255, 0/255, 0/255)
	)
end

# ╔═╡ dbc39958-c5b1-437c-a95b-a92bd630242d
astardf = createdf(readbenchmark.(glob("../benchmarks/results/rand*8*8*_AStar")))

# ╔═╡ 5acb45fa-7f1b-4433-80bf-ccaeddd06fe1
astargrouped = groupdf(astardf)

# ╔═╡ d4e9278b-09a9-4d9f-b22b-007b52c1d439
srateplot = let
	df = vcat(cbsgrouped, mstargrouped, astargrouped)

	ctg = CategoricalArray(df.algorithm)
	levels!(ctg, ["CBS", "MStar", "AStar"])
	
	@df df plot(:nrobots, :nsuccess, group = ctg,
		linewidth = 2,
		palette = :Set2_3,
		marker = (:diamond, 6),
		markerstrokecolor=[RGB(44/255, 162/255, 95/255) RGB(227/255, 75/255, 51/255) RGB(49/255, 130/255, 189/255)],
		xticks=4:16, xlabel=string(L"n", " (robots)"),
		yticks=0:2:20, ylabel="Solved Instances",
		labels = ["CBS" "M*" "A*"], legend = false,
		title = "(b) Success Rate"
	)
end

# ╔═╡ d54ddab5-aa17-409f-bdc1-2172ad61cf9b
rtimeplot = let
	df = vcat(cbsgrouped, mstargrouped, astargrouped)

	ctg = CategoricalArray(df.algorithm)
	levels!(ctg, ["CBS", "MStar", "AStar"])
	
	@df df[completecases(df), :] plot(:nrobots, :time, group = ctg,
		linewidth = 2,
		palette = :Set2_3,
		marker = (:diamond, 6),
		markerstrokecolor=[RGB(44/255, 162/255, 95/255) RGB(227/255, 75/255, 51/255) RGB(49/255, 130/255, 189/255)],		xticks=4:16, xlabel=string(L"n", " (robots)"),
		ylabel="Average Runtime (seconds)",
		labels = ["CBS" "M*" "A*"], legend = :topright,
		title = "(a) Runtime"
	)
end

# ╔═╡ b4f57efd-85cd-448e-9890-a89be8828644
begin
	savefig(rtimeplot, "rtimeplot.pdf")
	savefig(srateplot, "srateplot.pdf")
	savefig(avgkplot, "avgkplot.pdf")
	savefig(obvkplot, "obvkplot.pdf")
	savefig(clustkplot, "clustkplot.pdf")
	savefig(clustsrateplot, "clustsrateplot.pdf")
end

# ╔═╡ 0d61ee78-fa13-4d40-b64b-d719fe3b2766
insts = readinstance.(glob("../instances/new/rand*8x8*"))

# ╔═╡ 33c36cc9-d514-49c7-920f-7f191ff3310c
indvsols = Solution[
	plantosolution([ astar(r.pos, r.target, inst) for r in inst.robots ])
	for inst in insts
]

# ╔═╡ 5029c6f7-9b8e-4f79-a021-250d8b76cf0a
clustdf = let
	df = DataFrame()
	df.nrobots    = map(i -> length(i.robots), insts)
	df.clustering = map(getclustering, insts)
	df.ncolls     = map(p -> length(findcollisions(p[1], p[2])), zip(indvsols, insts))

	df = filter(:nrobots => (n -> n <= 32), df)
	
	def = combine(
		groupby(df, [:nrobots, :clustering]),
		:ncolls => mean => :ncolls,
	)
end

# ╔═╡ 8fcf1967-2a60-49b5-8ec2-5433c7709159
clustcolls = let

	@df clustdf plot(:nrobots, :ncolls, groups = :clustering,
		palette = [
			RGB(204/255, 236/255, 230/255), 
			RGB(102/255, 194/255, 164/255), 
			RGB(35/255, 139/255, 69/255), 
			RGB(0/255, 109/255, 44/255)
		],
		xticks=4:4:32,
		markers = true,
		legend = :topleft,
		xlabel = string(L"n", " (robots)"),
		ylabel = "Collisions",
		title = "Collisions w.r.t. Clustering"
	)
end

# ╔═╡ 469bd527-31e8-40ad-b883-fabd38823073
savefig(clustcolls, "clustcolls.pdf")

# ╔═╡ a0c33bb2-8374-4a17-8ef5-fd61b9b0cbf0
maximum(cbsdf[completecases(cbsdf), :].ncolls)

# ╔═╡ f8e01114-74d5-4905-9335-50a7955d88fd
function groupbycolls(df)
	df[:, :group] = map(x -> div(x, 1), df.ncolls)
	combine(
		groupby(df, [:group]),
		:time => mean ∘ skipmissing => :time,
		:avgkappa => mean ∘ skipmissing => :avgavg,
		:algorithm
	)
end

# ╔═╡ 2842aebd-bfef-4564-88bc-2385911e2594
cbscolls = let
	df = groupbycolls(cbsdf)
	df[completecases(df), :]
end

# ╔═╡ 4c57b38c-523f-4423-b865-cffd83636ae5
mstarcolls = groupbycolls(mstardf)

# ╔═╡ e3c52837-0fac-443d-a80b-cbcd2089aa16
astarcolls = groupbycolls(astardf)

# ╔═╡ 066911a8-42ef-41f9-b63d-95e701b0b87a
rtimecollsplot = let
	df = vcat(cbscolls, mstarcolls, astarcolls)
	
	ctg = CategoricalArray(df.algorithm)
	levels!(ctg, ["CBS", "MStar", "AStar"])
	
	groups = unique(df.group)
	xlabs  = map(g -> Int(g * 4), groups)

	@df df[completecases(df), :] plot(:group, :time, group = ctg,
		linewidth = 2,
		palette = :Set2_3,
		labels = ["CBS" "M*" "A*"], legend = :bottomright,
		markers = (:diamond, 6),
		markerstrokecolor=[RGB(44/255, 162/255, 95/255) RGB(227/255, 75/255, 51/255) RGB(8/255, 81/255, 156/255)],
		xticks = 0:4:42,
		# xticks = (0:10, xlabs),
		xlabel = "(b) Number of Collisions",
		ylabel = "Average Runtime (seconds)",
		title ="Runtime w.r.t. Collisions"
	)
end

# ╔═╡ d404a03a-6c34-45c7-bd91-969164cc9dae
savefig(rtimecollsplot, "rtimecollsplot.pdf")

# ╔═╡ d89fcb7d-0263-4e20-bf9d-d125f2db99ce
kcollsplot = let
	df = vcat(cbscolls, mstarcolls)
	
	ctg = CategoricalArray(df.algorithm)
	levels!(ctg, ["CBS", "MStar"])
	
	groups = unique(df.group)
	xlabs  = map(g -> Int(g * 4), groups)

	@df df[completecases(df), :] plot(:group, :avgavg, group = ctg,
		linewidth = 2,
		palette = :Set2_3,
		markers = (:diamond, 6),
		markerstrokecolor=[RGB(44/255, 162/255, 95/255) RGB(227/255, 75/255, 51/255)],
		xlims = (0,25),
		labels = ["CBS" "M*"], legend = :topleft,
		xlabel = "Number of Collisions",
		ylabel = string(L"\hat{\kappa}", " (average coupling)"),
		title ="(a) Coupling w.r.t. Collisions"
	)
end

# ╔═╡ 1be3d6ef-0d99-49a2-b6c6-3888333dd8be
savefig(kcollsplot, "kcollsplot.pdf")

# ╔═╡ 15de6d03-2bb4-4b64-adc2-daebd7d608a6
sig(x) = 1 / (1 + exp(-x))

# ╔═╡ 7637ac75-ad4f-4391-8145-5fc564b11698
logit(p) = log(p / (1 - p))

# ╔═╡ 9d63667d-95d7-46eb-9719-f090c7f54a92
relent(c, n, mk, w, h) = let
	p = c / (n * mk)
	q = min(0.99, 12 * (n - 1) / 5 * h * w)
	p * log2(p / q) + (1 - p) * log2((1 - p) / (1 - q))
end

# ╔═╡ d69052ae-94f8-4d0e-9aa8-04aa5256ddc4
let
	gr()
	f(c, k, n, mk, w, h) = relent(c, n, mk, w, h) * sig((1024 - k) / 1024)

	surface(1:32, 1:1024, (x, y) -> f(x, y, 8, 8, 4, 4),
		c = cgrad(:matter, rev=true),
		legend = false,
		surfacealpha = 0.5,
		xticks = 0:4:32,
		yticks = 200:200:800,
		xlabel = string(L"H_c", " (collisions)"),
		ylabel = string(L"H_k", " (constraints)"),
		zlabel = L"f(H)",
		title = "ConstraintGA Fitness Function",
	)
end

# ╔═╡ e03046fa-4125-4609-b60e-7765b0e86068
fitplot = let
	gr()
	f(c, k, n, mk, w, h) = relent(c, n, mk, w, h) * sig((n - k) / k)

	surface(0:50, 0:8, (x, y) -> f(x, y, 8, 16, 8, 8),
		c = cgrad(:matter, rev=true),
		legend = false,
		surfacealpha = 0.5,
		xticks = 0:5:50,
		yticks = 1:8,
		xlabel = "Collisions",
		ylabel = "Coupling",
		zlabel = "Fitness",
		camera = (35, 50)
	)
end

# ╔═╡ 5dc784b7-29a4-46d3-8d46-64170211a4b2
savefig(fitplot, "fitplot.pdf")

# ╔═╡ d4832b15-bc9a-4836-a776-a6e1ac05dbcb
map(x -> relent(x, 8, 16, 8, 8), 0:10)

# ╔═╡ ee185aac-c73f-4d6a-b7d4-f0f3e1703abc
let
	f(c, k, n, mk, w, h) = relent(c, n, mk, w, h) 
end

# ╔═╡ c99835ec-3bab-4019-ac25-01aee314cc68
let
	inst = readinstance.(glob("../instances/new/rand*n*24*8x8*"))[1]
	naivesol = plantosolution([ astar(r.pos, r.target, inst) for r in inst.robots ])
	println(length(findcollisions(naivesol, inst)))
	f(sol, k, n, w, h) = begin
		colls = length(findcollisions(sol, inst))
		relent(colls, n, w, h) * (1 + sig(k / (n * w * h)))
	end

	f(naivesol, 1, 5, 8, 8)
end

# ╔═╡ Cell order:
# ╠═7b1ce480-880f-11ec-0efd-b7aaebe5995e
# ╠═c3aef5cd-025f-435c-8bd2-0339dc41e6fa
# ╠═f669f189-1236-4d98-bb06-42c1a6348822
# ╠═84906ae8-c171-4767-8f1e-c122d6d32858
# ╠═5ebead4c-59bc-41c9-917b-6294d3a480cf
# ╠═ba78572c-ed64-4905-bc88-3fab7fdf038b
# ╠═53fe147b-cc9e-4244-911c-21cb42a4c880
# ╠═dbc39958-c5b1-437c-a95b-a92bd630242d
# ╠═e9aa333d-8414-4df5-8084-e3bb1a7d904d
# ╠═771f3abf-f394-402b-82c5-f2bddd7f8e06
# ╠═2395fd23-6a72-4c54-882f-dc14e5617346
# ╠═5acb45fa-7f1b-4433-80bf-ccaeddd06fe1
# ╠═fe586b09-e023-4c88-b25a-5c75f35e4f59
# ╠═d4e9278b-09a9-4d9f-b22b-007b52c1d439
# ╠═d54ddab5-aa17-409f-bdc1-2172ad61cf9b
# ╠═a6e24e73-dc86-4b40-ad42-924545fe29ac
# ╠═a1aaec89-5345-42ae-b79d-ae0f79bd79ad
# ╠═63866a15-b48d-4855-85e2-3888567ec567
# ╠═41336393-d3c3-483f-8120-be1860e64b8b
# ╠═2824d793-1053-4b26-8c2a-37e305c7382e
# ╠═e82aba7d-8576-4721-8642-7639608ca255
# ╠═b4f57efd-85cd-448e-9890-a89be8828644
# ╠═5661c146-9ad4-4ed0-aef5-ca6c5a200976
# ╠═0d61ee78-fa13-4d40-b64b-d719fe3b2766
# ╠═33c36cc9-d514-49c7-920f-7f191ff3310c
# ╠═5029c6f7-9b8e-4f79-a021-250d8b76cf0a
# ╠═8fcf1967-2a60-49b5-8ec2-5433c7709159
# ╠═469bd527-31e8-40ad-b883-fabd38823073
# ╠═a0c33bb2-8374-4a17-8ef5-fd61b9b0cbf0
# ╠═f8e01114-74d5-4905-9335-50a7955d88fd
# ╠═2842aebd-bfef-4564-88bc-2385911e2594
# ╠═4c57b38c-523f-4423-b865-cffd83636ae5
# ╠═e3c52837-0fac-443d-a80b-cbcd2089aa16
# ╠═066911a8-42ef-41f9-b63d-95e701b0b87a
# ╠═d404a03a-6c34-45c7-bd91-969164cc9dae
# ╠═d89fcb7d-0263-4e20-bf9d-d125f2db99ce
# ╠═1be3d6ef-0d99-49a2-b6c6-3888333dd8be
# ╠═d69052ae-94f8-4d0e-9aa8-04aa5256ddc4
# ╠═e03046fa-4125-4609-b60e-7765b0e86068
# ╠═d4832b15-bc9a-4836-a776-a6e1ac05dbcb
# ╠═5dc784b7-29a4-46d3-8d46-64170211a4b2
# ╠═ee185aac-c73f-4d6a-b7d4-f0f3e1703abc
# ╠═15de6d03-2bb4-4b64-adc2-daebd7d608a6
# ╠═7637ac75-ad4f-4391-8145-5fc564b11698
# ╠═9d63667d-95d7-46eb-9719-f090c7f54a92
# ╠═c99835ec-3bab-4019-ac25-01aee314cc68
