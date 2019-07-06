package controllers

import MATRIX_E
import MATRIX_I2
import paths.Path
import Vector2D
import toHeading
import Pose
import kotlin.math.pow
import SkidSteerCommand
import motionprofile.MotionProfile
import motionprofile.TrapezoidalMotionProfile

class TimeVariantGVFController(path: Path, val k_delta: Double, val k_n: Double, val limits: MotionProfile.Limits): TimeVariantPathController(path) {
    val motionProfile = TrapezoidalMotionProfile(path.length(), limits);
    fun vectorAt(r: Vector2D, closestT: Double): Vector2D {
        return path.tangentVec(closestT).minus(path.nVec(r, closestT).scalarMul(k_n * path.error(path.levelSet(r, closestT))))
    }
    fun desiredHeadingVecDeriv(r: Vector2D, speed: Double, vector: Vector2D, curHeading: Vector2D, closestT: Double): Vector2D {
        val error = path.error(path.levelSet(r, closestT))
        val errDeriv = path.errorGradient(r, closestT).dot(curHeading) * speed

        val vecDerivA = MATRIX_E.subtract(MATRIX_I2.scalarMultiply(error * k_n)).multiply(path.hessian(r, closestT)).operate(curHeading.toApacheVec())
        var vecDeriv = Vector2D.fromApacheVec(vecDerivA).scalarMul(speed)
        vecDeriv = vecDeriv.minus(path.nVec(r, closestT).scalarMul(errDeriv * k_n))

        val desHeadingVecDeriv = MATRIX_I2.scalarMultiply(1.0 / vector.norm()).subtract(vector.outerProduct().scalarMultiply(1.0 / vector.norm().pow(3))).operate(vecDeriv.toApacheVec())

        return Vector2D.fromApacheVec(desHeadingVecDeriv)
    }
    fun desiredCurvature(r: Vector2D, speed: Double, curHeading: Vector2D, vector: Vector2D, desiredHeadingVec: Vector2D, closestT: Double): Double {

        return -1.0 * this.desiredHeadingVecDeriv(r, speed, vector, curHeading, closestT).dot(desiredHeadingVec.getRightNormal())
    }
    override fun curvatureControl(pose: Pose, time: Double): SkidSteerCommand {
        val t = motionProfile.getPosition(time) / path.length()
        val speed = motionProfile.getSpeed(time)

        val curHeading_ = pose.directionVector()
        val curHeading = curHeading_.normalized()
        val vector = vectorAt(pose, t)
        val desiredHeadingVec = vector.normalized()
        val angleDelta = toHeading(desiredHeadingVec.angle() - curHeading.angle())

        val curvature = desiredCurvature(pose, speed, curHeading, vector, desiredHeadingVec, t) - k_delta * angleDelta
        return SkidSteerCommand(speed, curvature)
    }
    override fun vectorControl(pose: Pose, time: Double): Vector2D {
        val t = motionProfile.getPosition(time) / path.length()
        val vector = vectorAt(pose, t)
        val desiredHeadingVec = vector.normalized()
        return desiredHeadingVec.scalarMul(motionProfile.getSpeed(time))
    }
}