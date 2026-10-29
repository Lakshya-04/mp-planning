---
type: Reference
title: motion_planner
description: Package README for the planning-algorithms-only package -- CartesianPlanner, OmplPlanner, and the world-geometry loader utility.
tags: [motion-planning, package-readme]
generated: { by: human:Lakshya-04, at: 2026-07-21T13:15:52Z }
status: stable
---

# motion_planner

Planning algorithms for the manipulation stack.

## Package scope

This package holds the planning **algorithms** only -- `CartesianPlanner`, `OmplPlanner`, the
`Planner` interface + POD types, and `world_model_loader_node` (injects a `.pcd`/`.ply` point cloud
into the planning scene as an octomap, for obstacle testing). The orchestration layer built on top
of these planners (`ManipulationManager`, the `ExecuteMotionSegments` action server,
`CoverageSource`/`CoverageFeasibilityAnalyzer`, `MotionValidator`, `execution_tracker_node`, and the
FSP/Cartesian planner benchmark tool -- `PlannerBenchmark`) lives in the separate
`manipulation_manager` package, which depends on this one (never the other way around) -- see
that package's own README for `motion_planner_test.launch.py` (the benchmark tool) and
`cartesian_test.launch.py` (the mission-level validator).
