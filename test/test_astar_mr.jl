using Test
using DataStructures

using MotionPlanning.Collisions
using MotionPlanning.Model
using MotionPlanning.SingleRobotPlanning
using MotionPlanning.MultiRobotPlanning.Metrics
using MotionPlanning.MultiRobotPlanning.AStar


@testset "A* Multi-Robot" begin

    @testset "2x2 no collisions" begin
        robots = [
            Robot(1, (0, 0), (0, 1)),
            Robot(2, (1, 1), (1, 0))
        ]

        instance = MRMPInstance("test", robots, Vector{Obstacle}())

        solution = astar_mr(instance)

        println(solution)

        @test length(solution) == 2
    end

    @testset "2x2 with collisions" begin
        robots = [
            Robot(1, (0, 0), (0, 1)),
            Robot(2, (0, 1), (1, 1))
        ]

        instance = MRMPInstance("test", robots, Vector{Obstacle}())

        solution = astar_mr(instance)

        @test !hascollisions(solution, instance)
    end

    @testset "3x3 with obstacles, no collisions" begin
        robots = [
            Robot(1, (0, 0), (0, 2)),
            Robot(2, (2, 2), (2, 0))
        ]

        obstacles = [ (2, 1) ]

        instance = MRMPInstance("test", robots, obstacles)

        solution = astar_mr(instance)

        @test !hascollisions(solution, instance)
    end

    @testset "3x3 with obstacles and collisions" begin
        robots = [
            Robot(1, (0, 0), (1, 1)),
            Robot(2, (2, 2), (2, 0))
        ]

        obstacles = [ (2, 1) ]

        instance = MRMPInstance("test", robots, obstacles)

        solution = astar_mr(instance)

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

        solution = astar_mr(instance)

        @test length(solution) == 5
    end
end
