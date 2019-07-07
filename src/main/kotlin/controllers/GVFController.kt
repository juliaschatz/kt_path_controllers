package controllers

import math.MATRIX_E
import math.MATRIX_I2
import paths.Path
import math.Vector2D
import math.toHeading
import math.Pose
import kotlin.math.pow

class GVFController(path: Path, val k_delta: Double, val k_n: Double): PathController(path) {
    var lastT = 0.0001
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
    override fun curvatureControl(pose: Pose, speed: Double, dt: Double): Double {
        val closestT = path.closestTOnPathTo(pose, lastT)
        val curHeading_ = pose.directionVector()
        val curHeading = curHeading_.normalized()
        val vector = vectorAt(pose, closestT)
        val desiredHeadingVec = vector.normalized()
        val angleDelta = toHeading(desiredHeadingVec.angle() - curHeading.angle())
        lastT = closestT

        return desiredCurvature(pose, speed, curHeading, vector, desiredHeadingVec, closestT) - k_delta * angleDelta
    }
    override fun vectorControl(pose: Pose, speed: Double, dt: Double): Vector2D {
        val closestT = path.closestTOnPathTo(pose, lastT)
        lastT = closestT
        val vector = vectorAt(pose, closestT)
        val desiredHeadingVec = vector.normalized()
        return desiredHeadingVec
    }
}