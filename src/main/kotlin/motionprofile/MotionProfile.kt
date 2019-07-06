package motionprofile

abstract class MotionProfile(val length: Double, val limits: MotionProfile.Limits) {
    abstract fun getTime(): Double
    abstract fun getPosition(time: Double): Double
    abstract fun getSpeed(time: Double): Double
    abstract fun getAccel(time: Double): Double

    data class Limits(val cruiseVel: Double, val accel: Double, val jerkLimit: Double)
}