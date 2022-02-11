### A Pluto.jl notebook ###
# v0.17.5

using Markdown
using InteractiveUtils

# ╔═╡ 3ffef9fa-793b-11ec-0da0-4f2cd2f2202f
begin
	using Pkg
	Pkg.activate(joinpath(@__DIR__, ".."))
end

# ╔═╡ 4342b8e9-4eb1-419e-86b7-fc32271ca7da
begin
	using Revise

	using MotionPlanning.Benchmarks.Generators
	using MotionPlanning.Model
	using MotionPlanning.IO

	using Glob
	using Images
	using OffsetArrays
	using Statistics
end

# ╔═╡ 5a174931-8a62-439a-8845-4ce3ce907884
function drawinst(instance)
	w, h = instance.dims
	img = OffsetArray(repeat([RGB(255/255)], instance.dims...), 0:w-1, 0:h-1)
	for r in instance.robots
		img[r.target...] = RGB(102 / 255, 194 / 255, 165 / 255)
	end
	for o in instance.obstacles
		img[o...] = RGB(0)
	end
	img
end

# ╔═╡ 12920506-6eae-4839-94aa-86f7d41da343
function magnify(image, factor::Int64)
    w, h = size(image) .* factor
    magnified = OffsetArray(
        fill(RGB(0, 0, 0), w, h),
        0:(w - 1),
        0:(h - 1)
    )

    for (i, pixel) in zip(CartesianIndices(image), image)
        x, y = Tuple(i)
        xstart = (x * factor)
        xrange = xstart:(xstart + factor - 1)
        ystart = (y * factor)
        yrange = ystart:(ystart + factor - 1)
        magnified[xrange, yrange] .= pixel
    end

    magnified
end

# ╔═╡ 707ec8c3-00fc-4a5a-9410-bbebddf21ee6
begin
	uniforminst   = generate(20, (8, 8), 0.0, "yeet")
	lowmidinst    = generate(20, (8, 8), 0.33, "yeet")
	highmidinst   = generate(20, (8, 8), 0.66, "yeet")
	clusteredinst = generate(20, (8, 8), 1.0, "yeet")
end

# ╔═╡ e3b17196-d97f-4442-b9a4-95e6f3079814
u = drawinst(uniforminst)

# ╔═╡ da62ce31-35be-4916-9f2c-115df5097fe9
l = drawinst(lowmidinst)

# ╔═╡ 5838d304-2755-4245-85ac-e7c649046109
h = drawinst(highmidinst)

# ╔═╡ 23e34feb-dfc2-4790-892a-d8f09572f721
c = drawinst(clusteredinst)

# ╔═╡ a8816ae2-ebff-4aac-8958-1dbb6fc525ab
begin
	save("c000.png", magnify(u, 16))
	save("c033.png", magnify(l, 16))
	save("c066.png", magnify(h, 16))
	save("c100.png", magnify(c, 16))
end

# ╔═╡ b161645b-8f1f-4889-b057-d603e6feaf27
spread(clusteredinst)

# ╔═╡ 64e2376e-fba7-4ffb-90af-3b46d5f4d932
begin
	uniforms   = [ generate(32, (16, 16), 0.0, "yeet") for _ in 1:100 ]
	clustereds = [ generate(32, (16, 16), 1.0, "yeet") for _ in 1:100 ]
end

# ╔═╡ 74fa6186-c24c-40b9-b6db-f0441e369536
mean(spread, uniforms)

# ╔═╡ f6c43ab4-e95f-43df-9132-5862f8ec7059
mean(spread, clustereds)

# ╔═╡ cc41dbf3-fd2b-4df9-b163-13fe1f6de3ac
inst = readinstance.(glob("../instances/new/rand*n096_d16x16_c000.instance.json"))[1]

# ╔═╡ aeb04454-6f62-46fc-86a8-02b3f4647aed
drawinst(inst)

# ╔═╡ Cell order:
# ╠═3ffef9fa-793b-11ec-0da0-4f2cd2f2202f
# ╠═4342b8e9-4eb1-419e-86b7-fc32271ca7da
# ╠═5a174931-8a62-439a-8845-4ce3ce907884
# ╠═12920506-6eae-4839-94aa-86f7d41da343
# ╠═707ec8c3-00fc-4a5a-9410-bbebddf21ee6
# ╠═e3b17196-d97f-4442-b9a4-95e6f3079814
# ╠═da62ce31-35be-4916-9f2c-115df5097fe9
# ╠═5838d304-2755-4245-85ac-e7c649046109
# ╠═23e34feb-dfc2-4790-892a-d8f09572f721
# ╠═a8816ae2-ebff-4aac-8958-1dbb6fc525ab
# ╠═b161645b-8f1f-4889-b057-d603e6feaf27
# ╠═64e2376e-fba7-4ffb-90af-3b46d5f4d932
# ╠═74fa6186-c24c-40b9-b6db-f0441e369536
# ╠═f6c43ab4-e95f-43df-9132-5862f8ec7059
# ╠═cc41dbf3-fd2b-4df9-b163-13fe1f6de3ac
# ╠═aeb04454-6f62-46fc-86a8-02b3f4647aed
