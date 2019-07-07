package controllers

import math.Vector2D
import paths.Path
import math.Pose
import math.SkidSteerCommand

abstract class TimeVariantPathController(val path: Path) {
    abstract fun curvatureControl(pose: Pose, time: Double): SkidSteerCommand
    abstract fun vectorControl(pose: Pose, time: Double): Vector2D
}