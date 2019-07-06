package controllers

import Vector2D
import paths.Path
import Pose
import SkidSteerCommand

abstract class TimeVariantPathController(val path: Path) {
    abstract fun curvatureControl(pose: Pose, time: Double): SkidSteerCommand
    abstract fun vectorControl(pose: Pose, time: Double): Vector2D
}