import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin

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

    override fun error(r: Vector2D, closestT: Double): Double {
        val x = r.x
        val y = r.y
        return (x - x0).pow(2) / a.pow(2) + (y - y0).pow(2) / b.pow(2) - 1
    }

    override fun errorGradient(r: Vector2D, closestT: Double): Vector2D {
        return nVec(r, closestT).scalarMul(this.error(r, closestT))
    }

    override fun nVec(r: Vector2D, closestT: Double): Vector2D {
        var vec = (r - Vector2D(x0, y0)).normalized()
        if (error(r, 0.0) < 0) {
            vec = vec.neg()
        }
        return vec
    }

}