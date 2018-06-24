import kotlin.math.cos
import kotlin.math.sin

class Biarc {
    abstract class BiarcPart {
        abstract fun r(t: Double): Vector2D
        abstract fun length(): Double
        abstract fun project(p: Vector2D): Double
    }
    class LineSegment(val begin: Vector2D, val end: Vector2D): BiarcPart() {
        private val line = end.minus(begin)
        override fun r(t: Double): Vector2D {
            return begin + line.scalarMul(t)
        }
        override fun project(p: Vector2D): Double {
            return (begin + p).dot(line)
        }
        override fun length(): Double {
            return line.norm()
        }
    }
    class ArcSegment(val center: Vector2D, val radius: Double, val startAngle: Double, val endAngle: Double): BiarcPart() {
        override fun r(t: Double): Vector2D {
            val ang = lerp(startAngle, endAngle, t)
            return center + Vector2D(cos(ang), sin(ang)).scalarMul(radius)
        }
        override fun length(): Double {
            return (endAngle - startAngle) * radius
        }
        override fun project(r: Vector2D): Double {
            val angle = (center - r).angle()
            if (angle !in this) {
                throw IllegalArgumentException("Unable to project onto arc")
            }
            return (angle - startAngle) / (endAngle - startAngle)
        }
        operator fun contains(angle: Double): Boolean {
            val t = (angle - startAngle) / (endAngle - startAngle)
            return t in 0.0..1.0
        }
    }
}