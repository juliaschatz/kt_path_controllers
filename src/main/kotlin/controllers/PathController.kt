package controllers

import Vector2D
import Path
import Pose

abstract class PathController(val path: Path) {
    abstract fun curvatureControl(pose: Pose, speed: Double, dt: Double): Double
    abstract fun vectorControl(pose: Pose, speed: Double, dt: Double): Vector2D
}