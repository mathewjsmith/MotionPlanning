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
getclustering(bm) = parse(Int, bm.instancename[end - 2 : end])

# ╔═╡ 84906ae8-c171-4767-8f1e-c122d6d32858
function replacenothing!(df)
	mapcols!(c -> replace(c, nothing => missing), df)
end

# ╔═╡ 5ebead4c-59bc-41c9-917b-6294d3a480cf
function createdf(bms)
	df  = DataFrame(bms)
	
	df.clustering = map(getclustering, bms)

	df = transform(df, :time => ByRow(t -> t == 600.0) => :timedout)

	df = replacenothing!(df)
		
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
		:timedout
	])
end

# ╔═╡ ba78572c-ed64-4905-bc88-3fab7fdf038b
mstardf = createdf(readbenchmark.(glob("../benchmarks/results/rand*8x8*MStar")))

# ╔═╡ 53fe147b-cc9e-4244-911c-21cb42a4c880
cbsdf = createdf(readbenchmark.(glob("../benchmarks/results/rand*8x8*_CBS")))

# ╔═╡ dbc39958-c5b1-437c-a95b-a92bd630242d
astardf = createdf(readbenchmark.(glob("../benchmarks/results/rand*8*8*_AStar")))

# ╔═╡ e9aa333d-8414-4df5-8084-e3bb1a7d904d
groupdf(df) = combine(
		groupby(df, :nrobots),
		:algorithm,
		:clustering,
		:time => mean ∘ skipmissing => :time,
		:avgkappa => mean ∘ skipmissing => :avgavg,
		:timedout => (ts -> sum([ !t for t in ts])) => :nsuccess,
)

# ╔═╡ 771f3abf-f394-402b-82c5-f2bddd7f8e06
cbsgrouped = groupdf(cbsdf)

# ╔═╡ 2395fd23-6a72-4c54-882f-dc14e5617346
mstargrouped = groupdf(mstardf)

# ╔═╡ 5acb45fa-7f1b-4433-80bf-ccaeddd06fe1
astargrouped = groupdf(astardf)

# ╔═╡ fe586b09-e023-4c88-b25a-5c75f35e4f59
theme(:dao)

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
		labels = ["CBS" "M*" "A*"],
		title = "Success Rate"
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
		labels = ["CBS" "M*" "A*"],
		title = "Runtime"
	)
end

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
		xlabel = string(L"n", " (robots)"),
		ylabel = string(L"\hat{\kappa}", " (average coupling)"),
		title = "Average Degree of Coupling"
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
		xlabel = string(L"n", " (robots)"),
		ylabel = string(L"\kappa", " (observed coupling)"),
		title = "Observed Degree of Coupling",
	)
end

# ╔═╡ 63866a15-b48d-4855-85e2-3888567ec567
cbscluster = combine(
	groupby(cbsdf, [:nrobots, :clustering]),
	:time => mean => :time,
	:timedout => (ts -> 4 * sum([ !t for t in ts])) => :nsuccess,
	:avgkappa => mean ∘ skipmissing => :avgavg
)

# ╔═╡ 41336393-d3c3-483f-8120-be1860e64b8b
mstarcluster = combine(
	groupby(mstardf, [:nrobots, :clustering]),
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
		labels = ["CBS" "M*" "A*"],
		xlabel = string(L"n", " (robots)"),
		ylabel = string(L"\hat{\kappa}", " (average coupling)"),
		title = "Average degree of doupling w.r.t. clustering"
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
		labels = ["CBS" "M*" "A*"],
		title = "Success rate w.r.t. clustering"
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

# ╔═╡ b4f57efd-85cd-448e-9890-a89be8828644
begin
	savefig(rtimeplot, "rtimeplot.pdf")
	savefig(srateplot, "srateplot.pdf")
	savefig(avgkplot, "avgkplot.pdf")
	savefig(obvkplot, "obvkplot.pdf")
	savefig(clustkplot, "clustkplot.pdf")
	savefig(clustsrateplot, "clustsrateplot.pdf")
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
