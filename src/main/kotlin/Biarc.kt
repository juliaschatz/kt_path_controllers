import kotlin.math.cos
import kotlin.math.sin

class Biarc {
    class LineSegment(val begin: Vector2D, val end: Vector2D) {
        private val line = end.minus(begin)
        fun r(t: Double): Vector2D {
            return begin + line.scalarMul(t)
        }
        fun project(p: Vector2D): Double {
            return (begin + p).dot(line)
        }
        fun length(): Double {
            return line.norm()
        }
    }
    class ArcSegment(val center: Vector2D, val radius: Double, val startAngle: Double, val endAngle: Double) {
        fun r(t: Double): Vector2D {
            val ang = lerp(startAngle, endAngle, t)
            return center + Vector2D(cos(ang), sin(ang)).scalarMul(radius)
        }
        fun length(): Double {
            return (endAngle - startAngle) * radius
        }
        fun project(r: Vector2D): Vector2D {
            val angle = (center - r).angle()
            if (angle !in this) {
                throw IllegalArgumentException("Unable to project onto arc")
            }
            return this.r((angle - startAngle) / (endAngle - startAngle))
        }
        operator fun contains(angle: Double): Boolean {
            val t = (angle - startAngle) / (endAngle - startAngle)
            return t in 0.0..1.0
        }
    }
}