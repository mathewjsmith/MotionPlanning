using Test
using MotionPlanning.Constraints
using MotionPlanning.Model
using MotionPlanning.SingleRobotPlanning

@testset "A* with constraints" begin

    @testset "waits when forced by constraints" begin
        robots = [
            Robot(1, (0, 0), (1, 1))
        ]

        constraints = Constraint[
            ClashConstraint(3, (1, 1), 1),
            ClashConstraint(4, (1, 1), 1)
        ]

        instance = MRMPInstance("test", robots, StaticObstacles())

        constmat = constraintstomatrix(constraints, (1, instance.dims..., 8))

        println(constmat)

        path = astar(robots[1].pos, robots[1].target, instance; constmat=constmat, r=1)

        println(path)

        @test length(path) == 5
    end

    @testset "obeys overlap constraints" begin
        robots = [
            Robot(1, (0, 0), (1, 1))
        ]

        constraints = Constraint[
            OverlapConstraint(2, (0, 0), (1, 0), 1)
        ]

        instance = MRMPInstance("test", robots, StaticObstacles())

        constmat = constraintstomatrix(constraints, (1, instance.dims..., 8))

        path = astar(robots[1].pos, robots[1].target, instance; constmat=constmat, r=1)

        println(path)

        @test path == [(0, 0), (0, 1), (1, 1)]
    end

end
