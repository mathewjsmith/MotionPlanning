using Test
using MotionPlanning.Model
using MotionPlanning.SingleRobotPlanning

@testset "Extension Planner Algorithm" begin

    @testset "finds a shortest path when a path exists" begin
        robots = Vector{Robot}([
            Robot(1, (1, 1), (2, 2))
            Robot(2, (0, 0), (1, 1))
            Robot(3, (0, 1), (0, 2))
        ])

        instance = MRMPInstance("test", robots, Vector{Obstacle}())

        @test length(ep(robots[1].pos, robots[1].target, instance)) == manhattandist(robots[1].pos, robots[1].target) + 1
    end

    @testset "returns nothing when there is no path" begin
        robots = [
            Robot(1, (1, 1), (2, 2))
        ]

        obstacles = [
            (0, 0), (0, 1), (0, 2),
            (1, 0),         (1, 2),
            (2, 0), (2, 1), (2, 2)
        ]

        instance = MRMPInstance("test", robots, obstacles)

        @test ep(robots[1].pos, robots[1].target, instance) === nothing
    end

    @testset "works with dynamic obstacles" begin
        robots = [
            Robot(1, (0, 0), (1, 1))
        ]

        obstacles = DynamicObstacles([
            [(1, 1)],
            [(0, 1)],
            [(0, 0)]
        ])

        instance = MRMPInstance("test", robots, obstacles)

        @test length(ep(robots[1].pos, robots[1].target, instance)) == 3
    end

    @testset "cluster of five" begin
        robots = [
            Robot(1, (1, 0), (1, 2)),
        ]

        obstacles = DynamicObstacles([
            [(1,1), (0,1), (2,1), (1,2)],
            [(1,1), (0,2), (2,0), (2,2)],
            [(1,1), (1,2), (1,0), (2,1)],
            [(1,1), (2,2), (0,0), (2,0)],
            [(1,1), (2,1), (0,1), (1,0)]
        ])

        instance = MRMPInstance("test", robots, obstacles)
        instance.bounded = false

        path = ep(robots[1].pos, robots[1].target, instance)

        @test length(path) == 5
    end
end
