package controllers

import motionprofile.MotionProfile
import paths.Path
import math.Pose
import math.Twist2D
import motionprofile.TrapezoidalMotionProfile
import math.toHeading
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

class RamseteController(path: Path, val b: Double, val zeta: Double, val limits: MotionProfile.Limits): TwistController(path) {
    val motionProfile = TrapezoidalMotionProfile(path.length(), limits)

    override fun twistControl(pose: Pose, time: Double): Twist2D {
        val t = motionProfile.getPosition(time) / path.length()
        val pos = path.calculatePoint(t)
        val speed = motionProfile.getSpeed(time)
        val angVel: Double
        val angle = toHeading(path.tangentVec(t).angle())
        val cangle = toHeading(pose.heading)

        val dangle = toHeading(angle - cangle)
        val dx = pos.x - pose.x
        val dy = pos.y - pose.y

        val dtime = 0.001

        if (t < dtime) {
            val ot = motionProfile.getPosition(time + dtime) / path.length()
            angVel = (path.tangentVec(ot).angle() - angle) / dtime
        }
        else {
            val ot = motionProfile.getPosition(time - dtime) / path.length()
            angVel = (angle - path.tangentVec(ot).angle()) / dtime
        }

        val k1 = 2 * zeta * sqrt(angVel * angVel + b * speed * speed)
        val sinc = sin(dangle) / dangle

        val v = speed * cos(dangle) + k1 * (dx * cos(cangle) + dy * sin(cangle))
        val w = angVel + b * speed * sinc * (dy * cos(cangle) - dx * sin(cangle)) + k1 * dangle

        return Twist2D(v, w)
    }

}