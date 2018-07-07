import org.apache.commons.math3.linear.Array2DRowRealMatrix
import org.apache.commons.math3.linear.RealMatrix
import kotlin.math.*

class EllipsePath(val x0: Double, val y0: Double, val a: Double, val b: Double): Path() {
    override fun closestTOnPathTo(r: Vector2D): Double {
        return (r - Vector2D(x0, y0)).angle() / (2 * PI)
    }

    override fun calculatePoint(t: Double): Vector2D {
        val ang = 2 * PI * t
        return Vector2D(x0 + a * cos(ang), y0 + b * sin(ang))
    }

    override fun tangentVec(t: Double): Vector2D {
        val ang = 2 * PI * t
        return Vector2D(-a * sin(ang), b * cos(ang)).normalized()
    }

    override fun levelSet(r: Vector2D, closestT: Double): Double {
        val x = r.x
        val y = r.y
        return (x - x0).pow(2) / a.pow(2) + (y - y0).pow(2) / b.pow(2) - 1
    }

    override fun errorGradient(r: Vector2D, closestT: Double): Vector2D {
        return nVec(r, closestT).scalarMul(this.errorDeriv(this.levelSet(r, closestT)))
    }

    override fun error(s: Double): Double {
        return s
    }

    override fun errorDeriv(s: Double): Double {
        return 1.0
    }

    override fun hessian(r: Vector2D, closestT: Double): RealMatrix {
        return Array2DRowRealMatrix(arrayOf(doubleArrayOf(2.0, 0.0), doubleArrayOf(0.0, 2.0)))
    }

    override fun nVec(r: Vector2D, closestT: Double): Vector2D {
        return tangentVec(closestT).getRightNormal()
    }

}