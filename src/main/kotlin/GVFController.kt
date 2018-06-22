import kotlin.math.acos
import kotlin.math.pow

class GVFController(val path: Path, val k_delta: Double, val k_n: Double) {
    fun vectorAt(r: Vector2D): Vector2D {
        return path.t_vec(r).minus(path.n_vec(r).scalarMul(k_n * path.error(r)))
    }
    fun desiredHeadingVecDeriv(r: Vector2D, speed: Double, vector: Vector2D, curHeading: Vector2D): Vector2D {
        val error = path.error(r)
        val errDeriv = path.errorGradient(r).dot(curHeading) * speed

        val vecDerivA = MATRIX_E.subtract(MATRIX_I2.scalarMultiply(error * k_n)).multiply(path.hessian(r)).operate(curHeading.toApacheVec())
        var vecDeriv = Vector2D.fromApacheVec(vecDerivA).scalarMul(speed)
        vecDeriv = vecDeriv.minus(path.n_vec(r).scalarMul(errDeriv * k_n))

        val desHeadingVecDeriv = MATRIX_I2.scalarMultiply(1 / vector.norm()).subtract(vector.outerProduct().scalarMultiply(vector.norm().pow(3))).operate(vecDeriv.toApacheVec())

        return Vector2D.fromApacheVec(desHeadingVecDeriv)
    }
    fun desiredCurvature(r: Vector2D, speed: Double, curHeading: Vector2D, vector: Vector2D, desiredHeadingVec: Vector2D): Double {

        return -1.0 * this.desiredHeadingVecDeriv(r, speed, vector, curHeading).dot(desiredHeadingVec.getRightNormal())
    }
    fun curvatureControl(r: Vector2D, speed: Double, curHeading_: Vector2D): Double {
        val curHeading = curHeading_.normalized()
        val vector = vectorAt(r)
        val desiredHeadingVec = vector.normalized()
        val angleDelta = acos(desiredHeadingVec.dot(curHeading))

        return desiredCurvature(r, speed, curHeading, vector, desiredHeadingVec) - k_delta * angleDelta
    }
}