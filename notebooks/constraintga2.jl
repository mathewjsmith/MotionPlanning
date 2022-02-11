### A Pluto.jl notebook ###
# v0.17.5

using Markdown
using InteractiveUtils

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

# ╔═╡ c1dd0490-b020-4469-baee-c62aa2e91800
using Distributed

# ╔═╡ 22b8f8e0-72f4-11ec-0c12-3f9362497f6c
@everywhere begin
	using Pkg
	Pkg.activate(joinpath(@__DIR__, ".."))
end

# ╔═╡ 20769858-4b64-4944-b66d-3f3678862994
@everywhere	using MotionPlanning.GeneticAlgorithms.ConstraintGA2

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


# ╔═╡ 00000000-0000-0000-0000-000000000001
PLUTO_PROJECT_TOML_CONTENTS = """
[deps]
Distributed = "8ba89e20-285c-5b6f-9357-94700520ee1b"
Glob = "c27321d9-0574-5035-807b-f59d2c89b15c"
Pkg = "44cfe95a-1eb2-52ea-b672-e2afdf69b78f"
Revise = "295af30f-e4ad-537b-8983-00126c2a3abe"

[compat]
Glob = "~1.3.0"
Revise = "~3.3.1"
"""

# ╔═╡ 00000000-0000-0000-0000-000000000002
PLUTO_MANIFEST_TOML_CONTENTS = """
# This file is machine-generated - editing it directly is not advised

[[ArgTools]]
uuid = "0dad84c5-d112-42e6-8d28-ef12dabb789f"

[[Artifacts]]
uuid = "56f22d72-fd6d-98f1-02f0-08ddc0907c33"

[[Base64]]
uuid = "2a0f44e3-6c83-55bd-87e4-b1978d98bd5f"

[[CodeTracking]]
deps = ["InteractiveUtils", "UUIDs"]
git-tree-sha1 = "9aa8a5ebb6b5bf469a7e0e2b5202cf6f8c291104"
uuid = "da1fd8a2-8d9e-5ec2-8556-3022fb5608a2"
version = "1.0.6"

[[Dates]]
deps = ["Printf"]
uuid = "ade2ca70-3891-5945-98fb-dc099432e06a"

[[Distributed]]
deps = ["Random", "Serialization", "Sockets"]
uuid = "8ba89e20-285c-5b6f-9357-94700520ee1b"

[[Downloads]]
deps = ["ArgTools", "LibCURL", "NetworkOptions"]
uuid = "f43a241f-c20a-4ad4-852c-f6b1247861c6"

[[FileWatching]]
uuid = "7b1f6079-737a-58dc-b8bc-7a2ca5c1b5ee"

[[Glob]]
git-tree-sha1 = "4df9f7e06108728ebf00a0a11edee4b29a482bb2"
uuid = "c27321d9-0574-5035-807b-f59d2c89b15c"
version = "1.3.0"

[[InteractiveUtils]]
deps = ["Markdown"]
uuid = "b77e0a4c-d291-57a0-90e8-8db25a27a240"

[[JuliaInterpreter]]
deps = ["CodeTracking", "InteractiveUtils", "Random", "UUIDs"]
git-tree-sha1 = "b55aae9a2bf436fc797d9c253a900913e0e90178"
uuid = "aa1ae85d-cabe-5617-a682-6adf51b2e16a"
version = "0.9.3"

[[LibCURL]]
deps = ["LibCURL_jll", "MozillaCACerts_jll"]
uuid = "b27032c2-a3e7-50c8-80cd-2d36dbcbfd21"

[[LibCURL_jll]]
deps = ["Artifacts", "LibSSH2_jll", "Libdl", "MbedTLS_jll", "Zlib_jll", "nghttp2_jll"]
uuid = "deac9b47-8bc7-5906-a0fe-35ac56dc84c0"

[[LibGit2]]
deps = ["Base64", "NetworkOptions", "Printf", "SHA"]
uuid = "76f85450-5226-5b5a-8eaa-529ad045b433"

[[LibSSH2_jll]]
deps = ["Artifacts", "Libdl", "MbedTLS_jll"]
uuid = "29816b5a-b9ab-546f-933c-edad1886dfa8"

[[Libdl]]
uuid = "8f399da3-3557-5675-b5ff-fb832c97cbdb"

[[Logging]]
uuid = "56ddb016-857b-54e1-b83d-db4d58db5568"

[[LoweredCodeUtils]]
deps = ["JuliaInterpreter"]
git-tree-sha1 = "6b0440822974cab904c8b14d79743565140567f6"
uuid = "6f1432cf-f94c-5a45-995e-cdbf5db27b0b"
version = "2.2.1"

[[Markdown]]
deps = ["Base64"]
uuid = "d6f4376e-aef5-505a-96c1-9c027394607a"

[[MbedTLS_jll]]
deps = ["Artifacts", "Libdl"]
uuid = "c8ffd9c3-330d-5841-b78e-0817d7145fa1"

[[MozillaCACerts_jll]]
uuid = "14a3606d-f60d-562e-9121-12d972cd8159"

[[NetworkOptions]]
uuid = "ca575930-c2e3-43a9-ace4-1e988b2c1908"

[[OrderedCollections]]
git-tree-sha1 = "85f8e6578bf1f9ee0d11e7bb1b1456435479d47c"
uuid = "bac558e1-5e72-5ebc-8fee-abe8a469f55d"
version = "1.4.1"

[[Pkg]]
deps = ["Artifacts", "Dates", "Downloads", "LibGit2", "Libdl", "Logging", "Markdown", "Printf", "REPL", "Random", "SHA", "Serialization", "TOML", "Tar", "UUIDs", "p7zip_jll"]
uuid = "44cfe95a-1eb2-52ea-b672-e2afdf69b78f"

[[Printf]]
deps = ["Unicode"]
uuid = "de0858da-6303-5e67-8744-51eddeeeb8d7"

[[REPL]]
deps = ["InteractiveUtils", "Markdown", "Sockets", "Unicode"]
uuid = "3fa0cd96-eef1-5676-8a61-b3b8758bbffb"

[[Random]]
deps = ["Serialization"]
uuid = "9a3f8284-a2c9-5f02-9a11-845980a1fd5c"

[[Requires]]
deps = ["UUIDs"]
git-tree-sha1 = "838a3a4188e2ded87a4f9f184b4b0d78a1e91cb7"
uuid = "ae029012-a4dd-5104-9daa-d747884805df"
version = "1.3.0"

[[Revise]]
deps = ["CodeTracking", "Distributed", "FileWatching", "JuliaInterpreter", "LibGit2", "LoweredCodeUtils", "OrderedCollections", "Pkg", "REPL", "Requires", "UUIDs", "Unicode"]
git-tree-sha1 = "2f9d4d6679b5f0394c52731db3794166f49d5131"
uuid = "295af30f-e4ad-537b-8983-00126c2a3abe"
version = "3.3.1"

[[SHA]]
uuid = "ea8e919c-243c-51af-8825-aaa63cd721ce"

[[Serialization]]
uuid = "9e88b42a-f829-5b0c-bbe9-9e923198166b"

[[Sockets]]
uuid = "6462fe0b-24de-5631-8697-dd941f90decc"

[[TOML]]
deps = ["Dates"]
uuid = "fa267f1f-6049-4f14-aa54-33bafae1ed76"

[[Tar]]
deps = ["ArgTools", "SHA"]
uuid = "a4e569a6-e804-4fa4-b0f3-eef7a1d5b13e"

[[UUIDs]]
deps = ["Random", "SHA"]
uuid = "cf7118a7-6976-5b1a-9a39-7adc72f591a4"

[[Unicode]]
uuid = "4ec0a83e-493e-50e2-b9ac-8f72acf5a8f5"

[[Zlib_jll]]
deps = ["Libdl"]
uuid = "83775a58-1f1d-513f-b197-d71354ab007a"

[[nghttp2_jll]]
deps = ["Artifacts", "Libdl"]
uuid = "8e850ede-7688-5339-a07c-302acd2aaf8d"

[[p7zip_jll]]
deps = ["Artifacts", "Libdl"]
uuid = "3f19e933-33d8-53b3-aaab-bd5110c3b7a0"
"""

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
# ╟─00000000-0000-0000-0000-000000000001
# ╟─00000000-0000-0000-0000-000000000002
