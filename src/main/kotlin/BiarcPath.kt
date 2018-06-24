import org.apache.commons.math3.linear.RealMatrix
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.acos
import kotlin.math.pow

private fun biarcInterpolate(p1: Pose, p2: Pose): Pair<Biarc.BiarcPart, Biarc.BiarcPart> {
    val v = p1 - p2
    val t1 = p1.directionVector()
    val t2 = p2.directionVector()
    val t = t1 + t2
    val discriminant = v.dot(t).pow(2) + 2 * (1 - t1.dot(t2)) * v.dot(v)
    val denom = 2 * (1 - t1.dot(t2))
    val d2: Double
    if (t1 == t2) {
        d2 = v.dot(v) / (4 * v.dot(t2))
    }
    else if (denom == 0.0) {
        val pm = p1 + v.scalarMul(0.5)
        val c1 = p1 + v.scalarMul(0.25)
        val c2 = p1 + v.scalarMul(0.75)
        val r = v.norm() / 4
        val theta1 = if (v.zProd(t2) < 0) PI else -PI
        val theta2 = if (v.zProd(t2) > 0) PI else -PI
        val start1 = (p1 - pm).angle()
        val end1 = start1 + theta1
        val start2 = (pm - p2).angle()
        val end2 = start2 + theta2

        return Pair(Biarc.ArcSegment(c1, r, start1, end1), Biarc.ArcSegment(c2, r, start2, end2))
    }
    else {
        d2 = (-(v.dot(t)) + discriminant.pow(0.5)) / denom
    }
    val pm = (p1 + p2 + (t1 - t2).scalarMul(d2)).scalarMul(0.5)

    fun calcHalfBiarc(t: Vector2D, p: Vector2D, beginAngle: Double): Biarc.BiarcPart {
        val n = t.getLeftNormal()
        val pmp1 = pm - p
        if (pmp1.dot(n) == 0.0) {
            return Biarc.LineSegment(p, pm)
        }
        else {
            val s = (pmp1).dot(pmp1) / (n.scalarMul(2.0).dot(pmp1))
            val c = p + n.scalarMul(s)

            if (s == 0.0) {
                return Biarc.ArcSegment(c, s.absoluteValue, 0.0, 0.0)
            }
            val r = s.absoluteValue
            val op = (p - c).scalarMul(1 / r)
            val om = (pm - c).scalarMul(1 / r)
            val zProd = op.zProd(om)
            val theta: Double
            if (d2 > 0 && zProd > 0) {
                theta = acos(op.dot(om))
            }
            else if (d2 > 0 && zProd <= 0) {
                theta = -acos(op.dot(om))
            }
            else if (d2 <= 0 && zProd > 0) {
                theta = -2*PI + acos(op.dot(om))
            }
            else {
                theta = 2*PI - acos(op.dot(om))
            }

            return Biarc.ArcSegment(c, r, beginAngle, beginAngle + theta)
        }
    }

    return Pair(calcHalfBiarc(t1, p1, (p1 - pm).angle()), calcHalfBiarc(t2, p2, (pm - p2).angle()))
}

class BiarcPath(waypoints: Array<Pose>) : Path() {
    init {
        for (i in 0..waypoints.size-2) {
            val wp1 = waypoints.get(i)
            val wp2 = waypoints.get(i+1)
        }
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