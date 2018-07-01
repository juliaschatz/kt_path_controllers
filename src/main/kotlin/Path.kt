import org.apache.commons.math3.linear.RealMatrix

abstract class Path {
    abstract fun closestTOnPathTo(r: Vector2D): Double
    abstract fun calculatePoint(t: Double): Vector2D
    abstract fun tangentVec(t: Double): Vector2D
    abstract fun error(r: Vector2D, closestT: Double): Double
    abstract fun errorGradient(r: Vector2D, closestT: Double): Vector2D
    abstract fun hessian(r: Vector2D, closestT: Double): RealMatrix
    abstract fun n_vec(r: Vector2D, closestT: Double): Vector2D
    fun t_vec(r: Vector2D, closestT: Double): Vector2D {
        val n = this.n_vec(r, closestT)
        return n.getRightNormal()
    }
}