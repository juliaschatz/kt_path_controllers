import org.apache.commons.math3.linear.RealMatrix

abstract class Path {
    abstract fun error(r: Vector2D): Double
    abstract fun errorGradient(r: Vector2D): Vector2D
    abstract fun hessian(r: Vector2D): RealMatrix
    abstract fun n_vec(r: Vector2D): Vector2D
    fun t_vec(r: Vector2D): Vector2D {
        val n = this.n_vec(r)
        return n.getRightNormal()
    }
}