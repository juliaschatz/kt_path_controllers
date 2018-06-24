import kotlin.math.cos
import kotlin.math.sin

class Pose(override val x: Double, override val y: Double, val heading: Double): Vector2D(x, y) {
    fun directionVector(): Vector2D {
        return Vector2D(cos(heading), sin(heading)).normalized()
    }

}