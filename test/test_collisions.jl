using Test
using MotionPlanning.Collisions
using MotionPlanning.Model


@testset "Collision Detection" begin
    @testset "detects clashes" begin
        robots = [
            Robot(1, (0,0), (0,0))
            Robot(2, (0,0), (0,0))
        ]

        instance = MRMPInstance("test", robots, Vector{Obstacle}())

        plan = [[(0,0), (0, 0)], [(0, 0), (0, 0)]]
        
        @test hascollisions(plan, instance)
    end

    @testset "detects overlaps" begin
        robots = [
            Robot(1, (0, 0), (1, 0))
            Robot(2, (0, 1), (0, 0))
        ]

        instance = MRMPInstance("test", robots, Vector{Obstacle}())

        plan = [[(0,0), (0, 1)], [(1, 0), (0, 0)]]

        @test hascollisions(plan, instance)
    end

    @testset "does not detect collisions when there are none" begin
        robots = [
            Robot(1, (0, 0), (1, 0))
            Robot(2, (1, 1), (0, 1))
        ]
        
        instance = MRMPInstance("test", robots, Vector{Obstacle}())

        plan = [[(0, 0), (1, 1)], [(1, 0), (0, 1)]]

        @test !hascollisions(plan, instance)
    end

    @testset "detects clashes with dynamic obstacles" begin
        robots = [
            Robot(1, (0,0), (0,0))
        ]

        obstacles = DynamicObstacles([[(1, 0)], [(0, 0)]])

        instance = MRMPInstance("test", robots, obstacles)

        plan = [[(0,0)], [(0, 0)]]
        
        @test hascollisions(plan, instance)
    end

    @testset "detects overlaps with dynamic obstacles" begin
        robots = [
            Robot(1, (0, 0), (1, 0))
        ]

        obstacles = DynamicObstacles([[(1, 0)], [(1, 1)]])

        instance = MRMPInstance("test", robots, obstacles)

        plan = [[(0,0)], [(1, 0)]]
        
        @test hascollisions(plan, instance)
    end

    @testset "detects overlaps with dynamic obstacles where the obstacles is moving onto the source position" begin
        robots = [
            Robot(1, (0, 0), (1, 0))
        ]

        obstacles = DynamicObstacles([[(0, 1)], [(0, 0)]])

        instance = MRMPInstance("test", robots, obstacles)

        plan = [[(0,0)], [(1, 0)]]
        
        @test hascollisions(plan, instance)
    end

    @testset "does not detect clashes with dynamic obstacles when obstacles are ignored" begin
        robots = [
            Robot(1, (0,0), (0,0))
        ]

        obstacles = DynamicObstacles([[(1, 0)], [(0, 0)]])

        instance = MRMPInstance("test", robots, obstacles)

        plan = [[(0,0)], [(0, 0)]]
        
        @test !hascollisions(plan, instance; ignoreobstacles=true)
    end
end