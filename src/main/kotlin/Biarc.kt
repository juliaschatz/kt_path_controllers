import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.sin

class Biarc {
    abstract class BiarcPart {
        abstract fun r(t: Double): Vector2D
        abstract fun length(): Double
        abstract fun project(p: Vector2D): Double
        abstract fun tangentVec(t: Double): Vector2D
    }
    class LineSegment(val begin: Vector2D, val end: Vector2D): BiarcPart() {
        private val line = end.minus(begin)
        override fun r(t: Double): Vector2D {
            return begin + line.scalarMul(t)
        }
        override fun project(p: Vector2D): Double {
            val t = (begin + p).dot(line)
            if (t in 0.0..1.0) {
                return t
            }
            throw IllegalArgumentException("Point outside projection domain")
        }
        override fun length(): Double {
            return line.norm()
        }
        override fun tangentVec(t: Double): Vector2D {
            return line.normalized()
        }
    }
    class ArcSegment(val center: Vector2D, val radius: Double, val startAngle: Double, val endAngle: Double): BiarcPart() {
        fun angleFromT(t: Double): Double {
            return lerp(startAngle, endAngle, t)
        }
        override fun r(t: Double): Vector2D {
            val ang = angleFromT(t)
            return center + Vector2D(cos(ang), sin(ang)).scalarMul(radius)
        }
        override fun length(): Double {
            return (endAngle - startAngle).absoluteValue * radius
        }
        override fun project(p: Vector2D): Double {
            val angle = (center - p).angle()
            val oppAngle = angle + PI
            val negAngle = angle - PI
            if (angle in this) {
                return invertAngle(angle)
            }
            else if (oppAngle in this) {
                return invertAngle(oppAngle)
            }
            else if (negAngle in this) {
                return invertAngle(negAngle)
            }
            throw IllegalArgumentException("Point outside projection domain")
        }
        fun invertAngle(angle: Double): Double {
            return invLerp(startAngle, endAngle, angle)
        }

        override fun toString(): String {
            return "ArcSegment($center, $radius, $startAngle, $endAngle)"
        }

        override fun tangentVec(t: Double): Vector2D {
            val ang = angleFromT(t)
            return Vector2D(-sin(ang), cos(ang))
        }
        operator fun contains(angle: Double): Boolean {
            return invertAngle(angle) in 0.0..1.0
        }
    }

    class BiarcPartWrapper(val wrapped: BiarcPart, val begin: Double, val end: Double) {
        fun length(): Double {
            return wrapped.length()
        }
        private fun toWrappedSpace(t: Double): Double {
            return invLerp(begin, end, t)
        }
        private fun fromWrappedSpace(t: Double): Double {
            return lerp(begin, end, t)
        }
        fun project(p: Vector2D): Double {
            return fromWrappedSpace(wrapped.project(p))
        }
        fun r(t: Double): Vector2D {
            return wrapped.r(toWrappedSpace(t))
        }
        fun tangentVec(t: Double): Vector2D {
            return wrapped.tangentVec(toWrappedSpace(t))
        }
    }
}