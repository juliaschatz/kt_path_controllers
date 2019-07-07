package controllers

import paths.Path
import Pose
import Twist2D

abstract class TwistController(val path: Path) {
    abstract fun twistControl(pose: Pose, time: Double): Twist2D
}