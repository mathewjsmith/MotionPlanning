using Test
using DataStructures

using MotionPlanning.Collisions
using MotionPlanning.Model
using MotionPlanning.MultiRobotPlanning.CBS
using MotionPlanning.MultiRobotPlanning.Metrics


@testset "CBS" begin
    @testset "2x2 no collisions" begin
        robots = [
            Robot(1, (0, 0), (0, 1)),
            Robot(2, (1, 1), (1, 0))
        ]

        instance = MRMPInstance("test", robots, Vector{Obstacle}())

        (solution, _) = cbs(instance)

        @test length(solution) == 2
        @test !hascollisions(solution, instance)
    end

    @testset "2x2 with collisions" begin
        robots = [
            Robot(1, (0, 0), (0, 1)),
            Robot(2, (0, 1), (1, 1))
        ]

        instance = MRMPInstance("test", robots, Vector{Obstacle}())

        (solution, _) = cbs(instance)

        @test length(solution) == 3
        @test !hascollisions(solution, instance)
    end

    @testset "cluster-of-five scenario" begin
        robots = [
            Robot(1, (1, 1), (1, 1)),
            Robot(2, (0, 1), (2, 1)),
            Robot(3, (2, 1), (0, 1)),
            Robot(4, (1, 0), (1, 2)),
            Robot(5, (1, 2), (1, 0)),
        ]

        instance = MRMPInstance("test,", robots, Vector{Obstacle}())
        instance.bounded = false

        (solution, _) = cbs(instance)

        @test length(solution) == 5
    end

    @testset "κ can be derived from the constraints" begin
        robots = [
            Robot(1, (1, 1), (1, 1)),
            Robot(2, (0, 1), (2, 1)),
            Robot(3, (2, 1), (0, 1)),
            Robot(4, (1, 0), (1, 2)),
            Robot(5, (1, 2), (1, 0)),
        ]

        instance = MRMPInstance("test,", robots, Vector{Obstacle}())
        instance.bounded = false

        (solution, constraints) = cbs(instance)

        (min, max, mean) = κ(solution, constraints, instance)

        println(mean)

        @test max <= 5
    end
end