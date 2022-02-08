### A Pluto.jl notebook ###
# v0.17.5

using Markdown
using InteractiveUtils

# ╔═╡ 8f0bc26e-884f-11ec-1bee-8da2b9ceb47b
begin
	using Pkg
	Pkg.activate(joinpath(@__DIR__, ".."))
end

# ╔═╡ 7894df26-be51-46b4-bbc3-e6d9e7805e3b
begin
	using Revise

	using MotionPlanning.IO
	using MotionPlanning.Benchmarks
	using MotionPlanning.Collisions
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

# ╔═╡ 2d3d1eef-bf57-40fe-8982-f45ee878d42a
getclustering(bm) = parse(Int, bm.instancename[end - 2 : end])

# ╔═╡ 4bff31f9-f878-4887-8a72-251ddff494e1
function replacenothing!(df)
	mapcols!(c -> replace(c, nothing => missing), df)
end

# ╔═╡ bee83836-15bc-4f77-b9ee-ae221e2e0a12
function createdf(bms)
	df  = DataFrame(bms)
	
	df.clustering = map(getclustering, bms)

	df = transform(df, :time => ByRow(t -> t == 600.0) => :timedout)

	df = replacenothing!(df)

	dudinst = MRMPInstance("", [], StaticObstacles(), (8, 8), false)
	
	df = transform(
		df, 
		:solution => ByRow(sol ->
			ismissing(sol) ? missing : length(findcollisions(sol, dudinst))
		) => :ncollisions
	)
		
	select(df, [
		:algorithm,
		:instancename, 
		:nrobots, 
		:time, 
		:makespan, 
		:totalmoves,
		:clustering,
		:timedout,
		:ncollisions
	])
end

# ╔═╡ a88d6c71-df0c-4a92-9ea7-1a7cd6030846
gadf = createdf(readbenchmark.(glob("../benchmarks/results/rand*8x8*ConstraintGA")))

# ╔═╡ ccb7686c-a335-4b32-a8f2-869bce7e2c27
srpdf = createdf(readbenchmark.(glob("../benchmarks/results/rand*8x8*IndvPaths")))

# ╔═╡ 5453ed15-955d-4121-9e94-1cc837edf4fd
cbsdf = createdf(readbenchmark.(glob("../benchmarks/results/rand*8x8*_CBS")))

# ╔═╡ fcbe5651-67b7-418e-bb2a-316f0a89b464
gacbsdf = createdf(readbenchmark.(glob("../benchmarks/results/rand*8x8*GA-CBS")))

# ╔═╡ 76b04f09-6511-4a34-96d8-6dac177529f3
collratiosdf = let
	df = innerjoin(
		gadf, select(srpdf, [:instancename, :ncollisions]),
		on = :instancename,
		makeunique=true
	)

	transform(df[completecases(df), :], 
		[:ncollisions, :ncollisions_1] => ByRow(
			(g, s) -> s == 0 ? 1 : 100 * (1 - (g / s))
		) => :collratio
	)
end

# ╔═╡ 3bf780ff-b7a0-49ce-94a4-b51ef6379978
groupedcollratios = combine(
	groupby(collratiosdf, :nrobots),
	:collratio => mean => :collratio,
	:time => mean ∘ skipmissing => :time,
)

# ╔═╡ 920db506-731f-44e4-afdd-a5da2923ec8d
theme(:dao)

# ╔═╡ 19447c0e-a925-476e-b7e0-5706dc4fcfe4
gacollratiosplot = @df collratiosdf boxplot(:nrobots, :collratio, 
	palette = :Set2_3, linewidth=1, outliers = false,
	linecolor = RGB(0/255, 109/255, 44/255),
	xticks = 4:4:32,
	legend = false,
	xlabel = string(L"n", " (robots)"),
	ylabel = "Collisions Resolved (%)",
	title = "Percentage of Collisions Resolved"
)

# ╔═╡ 8324e7a0-97b4-4adf-b9b3-347f9018fc84
garuntimeplot = @df groupedcollratios plot(:nrobots, :time,
	palette = :Set2_3, linewidth=2,
	marker = (:diamond, 8),	markerstrokewidth = 1, 
	markerstrokecolor=[RGB(44/255, 162/255, 95/255)],
	xticks = 4:4:32,
	legend = false,
	xlabel = string(L"n", " (robots)"),
	ylabel = "Runtime (seconds)",
	title = "Time to complete 1024 generations"
)

# ╔═╡ 737192de-3af2-4b83-bad6-45d3dbed9469
begin
	savefig(gacollratiosplot, "gacollratiosplot.pdf")
	savefig(garuntimeplot, "garuntimeplot.pdf")
end

# ╔═╡ b3c3e340-6e90-4e0d-a1f9-89a43d1dbd3c
makespanratiodf = let
	df = innerjoin(
		gadf, select(cbsdf, [:instancename, :makespan]),
		on = :instancename,
		makeunique=true
	)

	transform(df[completecases(df), :], 
		[:makespan, :makespan_1] => ByRow(
			(g, c) -> 100 * (1 - (g / c))
		) => :makespanratio
	)
end

# ╔═╡ 333e6c8b-db35-46ee-acd9-70ff0ddd864b
nsubopt = sum(map(r -> r > 0, makespanratiodf.makespanratio))

# ╔═╡ 3488cf1e-f45d-4567-821e-b0f6dc190670
avgsubopt = sum(filter(r -> r > 0, makespanratiodf.makespanratio)) / nsubopt

# ╔═╡ e14dbe31-6aae-4854-bf25-589c8b7f3ff9
size(makespanratiodf)

# ╔═╡ cc979a1d-f336-4ad9-817a-f7e203699e6c
groupdf(df) = combine(
		groupby(df, :nrobots),
		:algorithm,
		:clustering,
		:time => mean ∘ skipmissing => :time,
		:timedout => (ts -> sum([ !t for t in ts])) => :nsuccess,
)

# ╔═╡ 630d88e3-e733-47a9-ad41-03e3d94d018f
gacbsgrouped = groupdf(gacbsdf)

# ╔═╡ 1b968aa6-598d-47e9-b142-d9a0f41129f7
cbsgrouped = groupdf(cbsdf)

# ╔═╡ 45fd1685-07d9-4d25-96ca-a839e632da1b
srateplot = let
	df = vcat(cbsgrouped, gacbsgrouped)

	ctg = CategoricalArray(df.algorithm)
	levels!(ctg, ["CBS", "GA-CBS"])
	
	@df df plot(:nrobots, :nsuccess, group = ctg,
		linewidth = 2,
		palette = :Set2_3,
		marker = (:diamond, 6),
		xticks=4:16, xlabel=string(L"n", " (robots)"),
		yticks=0:2:20, ylabel="Solved Instances",
		labels = ["CBS" "GA-CBS"],
		title = "Success Rate"
	)
end

# ╔═╡ Cell order:
# ╠═8f0bc26e-884f-11ec-1bee-8da2b9ceb47b
# ╠═7894df26-be51-46b4-bbc3-e6d9e7805e3b
# ╠═2d3d1eef-bf57-40fe-8982-f45ee878d42a
# ╠═4bff31f9-f878-4887-8a72-251ddff494e1
# ╠═bee83836-15bc-4f77-b9ee-ae221e2e0a12
# ╠═a88d6c71-df0c-4a92-9ea7-1a7cd6030846
# ╠═ccb7686c-a335-4b32-a8f2-869bce7e2c27
# ╠═5453ed15-955d-4121-9e94-1cc837edf4fd
# ╠═fcbe5651-67b7-418e-bb2a-316f0a89b464
# ╠═76b04f09-6511-4a34-96d8-6dac177529f3
# ╠═3bf780ff-b7a0-49ce-94a4-b51ef6379978
# ╠═920db506-731f-44e4-afdd-a5da2923ec8d
# ╠═19447c0e-a925-476e-b7e0-5706dc4fcfe4
# ╠═8324e7a0-97b4-4adf-b9b3-347f9018fc84
# ╠═737192de-3af2-4b83-bad6-45d3dbed9469
# ╠═b3c3e340-6e90-4e0d-a1f9-89a43d1dbd3c
# ╠═333e6c8b-db35-46ee-acd9-70ff0ddd864b
# ╠═3488cf1e-f45d-4567-821e-b0f6dc190670
# ╠═e14dbe31-6aae-4854-bf25-589c8b7f3ff9
# ╠═cc979a1d-f336-4ad9-817a-f7e203699e6c
# ╠═630d88e3-e733-47a9-ad41-03e3d94d018f
# ╠═1b968aa6-598d-47e9-b142-d9a0f41129f7
# ╠═45fd1685-07d9-4d25-96ca-a839e632da1b
