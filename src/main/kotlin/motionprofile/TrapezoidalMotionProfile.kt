package motionprofile

import kotlin.math.sqrt

class TrapezoidalMotionProfile(length: Double, limits: MotionProfile.Limits) : MotionProfile(length, limits) {
    val cruiseTime: Double
    val accelTime: Double
    val maxSpeed: Double
    init {
        var accelTime_ = limits.cruiseVel / limits.accel // Time for one acceleration or deceleration
        var accelDist = 0.5 * limits.accel * accelTime_  * accelTime_ // Distance that one side of the acceleration will go
        if (2 * accelDist >= length) { // Triangular profile
            cruiseTime = 0.0
            accelDist = 0.5 * length
        }
        else {
            var cruiseDist = length - 2 * accelDist
            cruiseTime = cruiseDist / limits.cruiseVel
        }
        accelTime = sqrt(2 * accelDist / limits.accel)
        maxSpeed = accelTime * limits.accel
    }
    override fun getTime(): Double {
        return cruiseTime + 2 * accelTime
    }

    override fun getPosition(time: Double): Double {
        var sumPos = 0.0
        if (time <= 0.0) {
            return 0.0
        }
        if (time >= getTime()) {
            return length
        }
        if (time > accelTime) {
            sumPos += 0.5 * limits.accel * accelTime * accelTime
            if (time - accelTime < cruiseTime) {
                sumPos += (time - accelTime) * limits.cruiseVel
            }
            else {
                val sectTime = time - cruiseTime - accelTime
                sumPos += cruiseTime * limits.cruiseVel
                sumPos += (maxSpeed - 0.5 * limits.accel * sectTime) * sectTime
            }
        }
        else {
            sumPos += 0.5 * limits.accel * time * time;
        }
        return sumPos
    }

    override fun getSpeed(time: Double): Double {
        if (time < 0.0) {
            return 0.0
        }
        else if (time < accelTime) {
            return time * limits.accel
        }
        else if (time < accelTime + cruiseTime) {
            return limits.cruiseVel
        }
        else if (time < getTime()) {
            val sectTime = time - cruiseTime - accelTime
            return maxSpeed - sectTime * limits.accel
        }
        return 0.0
    }

    override fun getAccel(time: Double): Double {
        if (time < 0.0) {
            return 0.0
        }
        else if (time < accelTime) {
            return limits.accel
        }
        else if (time < accelTime + cruiseTime) {
            return 0.0
        }
        else if (time < getTime()) {
            return -limits.accel
        }
        return 0.0
    }
}