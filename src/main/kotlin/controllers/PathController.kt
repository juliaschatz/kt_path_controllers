package controllers

import math.Vector2D
import paths.Path
import math.Pose

abstract class PathController(val path: Path) {
    abstract fun curvatureControl(pose: Pose, speed: Double, dt: Double): Double
    abstract fun vectorControl(pose: Pose, speed: Double, dt: Double): Vector2D
}