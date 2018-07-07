import kotlin.math.pow

class GVFController(val path: Path, val k_delta: Double, val k_n: Double) {
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
    fun curvatureControl(r: Vector2D, speed: Double, curHeading_: Vector2D, dt: Double): Double {
        val closestT = path.closestTOnPathTo(r)
        val curHeading = curHeading_.normalized()
        val vector = vectorAt(r, closestT)
        val desiredHeadingVec = vector.normalized()
        val angleDelta = toHeading(desiredHeadingVec.angle() - curHeading.angle())

        return desiredCurvature(r, speed, curHeading, vector, desiredHeadingVec, closestT) - k_delta * angleDelta
    }
    fun vectorControl(r: Vector2D): Vector2D {
        val closestT = path.closestTOnPathTo(r)
        val vector = vectorAt(r, closestT)
        val desiredHeadingVec = vector.normalized()
        return desiredHeadingVec
    }
}