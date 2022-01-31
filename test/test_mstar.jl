using Test
using DataStructures

using MotionPlanning.Collisions
using MotionPlanning.Model
using MotionPlanning.SingleRobotPlanning
using MotionPlanning.MultiRobotPlanning.Metrics
using MotionPlanning.MultiRobotPlanning.MStar
using MotionPlanning.MultiRobotPlanning.Objectives
using MotionPlanning.MultiRobotPlanning.Heuristics


@testset "M*" begin

    @testset "2x2 no collisions" begin
        robots = [
            Robot(1, (0, 0), (0, 1)),
            Robot(2, (1, 1), (1, 0))
        ]

        instance = MRMPInstance("test", robots, Vector{Obstacle}())

        solution, _, _ = mstar(instance, MakeSpan(), MaxDist())

        @test !hascollisions(solution, instance)
    end

    @testset "2x2 with collisions" begin
        robots = [
            Robot(1, (0, 0), (0, 1)),
            Robot(2, (0, 1), (1, 1))
        ]

        instance = MRMPInstance("test", robots, Vector{Obstacle}())

        solution, _, _ = mstar(instance, MakeSpan(), MaxDist())

        @test !hascollisions(solution, instance)
    end

    @testset "3x3 with obstacles, no collisions" begin
        robots = [
            Robot(1, (0, 0), (0, 2)),
            Robot(2, (2, 2), (2, 0))
        ]

        obstacles = [ (2, 1) ]

        instance = MRMPInstance("test", robots, obstacles)

        solution, _, _ = mstar(instance, MakeSpan(), MaxDist())

        @test !hascollisions(solution, instance)
    end

    @testset "3x3 with obstacles and collisions" begin
        robots = [
            Robot(1, (0, 0), (1, 1)),
            Robot(2, (2, 2), (2, 0))
        ]

        obstacles = [ (2, 1) ]

        instance = MRMPInstance("test", robots, obstacles)

        solution, _, _ = mstar(instance, MakeSpan(), MaxDist())

        @test !hascollisions(solution, instance)
    end

    @testset "3x3 with obstacles and collisions pareto" begin
        robots = [
            Robot(1, (0, 0), (1, 1)),
            Robot(2, (2, 2), (2, 0))
        ]

        obstacles = [ (2, 1) ]

        instance = MRMPInstance("test", robots, obstacles)

        solution, _, _ = mstar(instance, MakeSpan(), ParetoMaxDistSIC())

        @test !hascollisions(solution, instance)
    end

    @testset "cluster-of-five scenario" begin
        robots = [
            Robot(1, (1, 1), (1, 1)),
            Robot(2, (0, 1), (2, 1)),
            Robot(1, (2, 1), (0, 1)),
            Robot(1, (1, 0), (1, 2)),
            Robot(1, (1, 2), (1, 0)),
        ]

        obstacles = StaticObstacles()

        instance = MRMPInstance("test,", robots, obstacles)

        solution, _, _ = mstar(instance, MakeSpan(), ParetoMaxDistSIC())

        @test length(solution) == 5
    end

    @testset "κ can be derived from the ConfigGraph" begin
        robots = [
            Robot(1, (1, 1), (1, 1)),
            Robot(2, (0, 1), (2, 1)),
            Robot(1, (2, 1), (0, 1)),
            Robot(1, (1, 0), (1, 2)),
            Robot(1, (1, 2), (1, 0)),
        ]

        obstacles = StaticObstacles()

        instance = MRMPInstance("test,", robots, obstacles)

        solution, G, path = mstar(instance, MakeSpan(), ParetoMaxDistSIC())
        
        min, max, mean = κ(G, path)

        println(mean)

        @test max <= 5
    end

end
