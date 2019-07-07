package controllers

import paths.Path
import math.Pose
import math.Twist2D

abstract class TwistController(val path: Path) {
    abstract fun twistControl(pose: Pose, time: Double): Twist2D
}