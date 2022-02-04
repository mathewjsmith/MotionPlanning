using Test
using DataStructures

using MotionPlanning.Collisions
using MotionPlanning.Model
using MotionPlanning.MultiRobotPlanning.Shadoks


@testset "Shadoks" begin
    @testset "2x2 no collisions" begin
        robots = [
            Robot(1, (0, 0), (0, 1)),
            Robot(2, (1, 1), (1, 0))
        ]

        instance = MRMPInstance("test", robots, Vector{Obstacle}())

        solution = shadoks(instance)

		@test !hascollisions(solution, instance; ignoreobstacles=true)
    end

    @testset "2x2 with collisions" begin
        robots = [
            Robot(1, (0, 0), (1, 1)),
            Robot(2, (1, 1), (0, 0))
        ]

        instance = MRMPInstance("test", robots, Vector{Obstacle}())

        solution = shadoks(instance)
        
		@test !hascollisions(solution, instance; ignoreobstacles=true)
    end

    @testset "cluster-of-five scenario" begin
        robots = [
            Robot(1, (1, 1), (1, 1)),
            Robot(2, (0, 1), (2, 1)),
            Robot(3, (2, 1), (0, 1)),
            Robot(4, (1, 0), (1, 2)),
            Robot(5, (1, 2), (1, 0)),
        ]

        obstacles = StaticObstacles()

        instance = MRMPInstance("test", robots, obstacles)
        instance.bounded = false

        solution = shadoks(instance)

        @test !hascollisions(solution, instance; ignoreobstacles=true)
    end
end
