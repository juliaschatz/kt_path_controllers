import kotlin.math.cos
import kotlin.math.sin

class Pose(override val x: Double, override val y: Double, val heading: Double): Vector2D(x, y) {
    fun directionVector(): Vector2D {
        return Vector2D(cos(heading), sin(heading)).normalized()
    }
    fun translate(r: Vector2D): Vector2D {
        val dx = r.x - x
        val dy = r.y - y
        val c = cos(heading)
        val s = sin(heading)
        return Vector2D(
                dx * c + dy * s,
                dx * -s + dy * c)
    }
    fun translate(p: Pose): Pose {
        val vec = translate(p as Vector2D)
        return Pose(vec.x, vec.y, p.heading - heading)
    }

    fun invTranslate(r: Vector2D): Vector2D {
        val c = cos(heading)
        val s = sin(heading)
        val dxp = r.x * c - r.y * s
        val dyp = r.x * s + r.y * c
        return Vector2D(x + dxp, y + dyp)
    }

    override fun toString(): String {
        return "Pose($x, $y, $heading)"
    }

}