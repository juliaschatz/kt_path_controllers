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

}