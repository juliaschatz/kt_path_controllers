import org.apache.commons.math3.linear.RealMatrix

class BiarcPath(wayponts: Array<Pose>) : Path() {
    init {

    }
    override fun error(r: Vector2D): Double {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun errorGradient(r: Vector2D): Vector2D {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun hessian(r: Vector2D): RealMatrix {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun n_vec(r: Vector2D): Vector2D {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
}