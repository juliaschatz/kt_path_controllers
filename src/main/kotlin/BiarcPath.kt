import org.apache.commons.math3.linear.Array2DRowRealMatrix
import org.apache.commons.math3.linear.RealMatrix
import java.lang.IllegalArgumentException
import kotlin.math.*

fun biarcInterpolate(p1: Pose, p2: Pose): Pair<Biarc.BiarcPart, Biarc.BiarcPart> {
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
        val start2 = (p2 - c2).angle()
        val end2 = start2 + theta2

        return Pair(Biarc.ArcSegment(c1, r, start1, end1), Biarc.ArcSegment(c2, r, start2, end2))
    }
    else {
        d2 = (-(v.dot(t)) + discriminant.pow(0.5)) / denom
    }
    val pm = (p1 + p2 + (t1 - t2).scalarMul(d2)).scalarMul(0.5)

    fun calcHalfBiarc(t: Vector2D, p: Vector2D, direction: Double): Biarc.BiarcPart {
        val n = t.getLeftNormal()
        val pmp1 = pm - p
        if (pmp1.dot(n) == 0.0) {
            return if (direction > 0) Biarc.LineSegment(p, pm) else Biarc.LineSegment(pm, p)
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
            val beginAngle: Double
            if (direction > 0) {
                beginAngle = (p - c).angle()
            }
            else {
                beginAngle = (pm - c).angle()
            }
            var endAngle = beginAngle + theta * direction
            endAngle = toHeading(endAngle)
            return Biarc.ArcSegment(c, r, beginAngle, endAngle)
        }
    }

    return Pair(calcHalfBiarc(t1, p1,1.0), calcHalfBiarc(t2, p2,-1.0))
}

class BiarcPath(val waypoints: Array<Pose>) : Path() {
    val segments = arrayOfNulls<Biarc.BiarcPartWrapper>((waypoints.size-1) * 2)
    val totalLen: Double
    init {
        val segmentsUnWrapped = arrayOfNulls<Biarc.BiarcPart>((waypoints.size-1)*2)
        for (i in 0..waypoints.size-2) {
            val wp1 = waypoints.get(i)
            val wp2 = waypoints.get(i+1)
            val (part1, part2) = biarcInterpolate(wp1, wp2)
            segmentsUnWrapped.set(2*i, part1)
            segmentsUnWrapped.set(2*i+1, part2)
        }
        totalLen = segmentsUnWrapped.sumByDouble { it!!.length() }
        var lastEnd = 0.0
        segmentsUnWrapped.forEachIndexed { i, biarcPart ->
            val begin = lastEnd
            val normalizedLen = biarcPart!!.length() / totalLen
            val end = begin + normalizedLen
            segments.set(i, Biarc.BiarcPartWrapper(biarcPart, begin, end))
            lastEnd = end
        }
    }

    private fun getSegment(t: Double): Biarc.BiarcPartWrapper {
        for (segment in segments) {
            if (segment == null) {
                continue
            }
            if (t in segment.begin..segment.end) {
                return segment
            }
        }
        throw IllegalArgumentException("t outside domain")
    }

    override fun closestTOnPathTo(r: Vector2D): Double {
        var minDist = Double.MAX_VALUE
        var minT: Double? = null
        segments.forEach {
            try {
                val testT = it!!.project(r)
                val testPt = it.r(testT)
                val testDist = testPt.sqDist(r)
                if (testDist < minDist) {
                    minDist = testDist
                    minT = testT
                }
            }
            catch (e: IllegalArgumentException) {
                // Do nothing, point outside projection domain
            }
        }
        if (minT == null) {
            throw IllegalArgumentException("Point outside projection domain")
        }
        return minT as Double
    }

    override fun calculatePoint(t: Double): Vector2D {
        return getSegment(t).r(t)
    }

    override fun tangentVec(t: Double): Vector2D {
        return getSegment(t).tangentVec(t)
    }

    override fun error(r: Vector2D, closestT: Double): Double {
        // Phi
        val pathPt = calculatePoint(closestT)
        val pathTangentVec = tangentVec(closestT)
        val ptPathVec = r - pathPt
        val sgn = sign(pathTangentVec.zProd(ptPathVec))
        return sgn * r.sqDist(pathPt)
    }

    override fun errorGradient(r: Vector2D, closestT: Double): Vector2D {
        return n_vec(r, closestT)
    }

    override fun hessian(r: Vector2D, closestT: Double): RealMatrix {
        // Discretely calculate the hessian because lol
        val d = 0.001
        val x1 = this.error(r - Vector2D(d, 0.0), closestT)
        val x2 = this.error(r, closestT)
        val x3 = this.error(r + Vector2D(d, 0.0), closestT)

        val dx1 = (x2 - x1) / d
        val dx2 = (x3 - x2) / d

        val ddx = (dx2 - dx1) / d

        val y1 = this.error(r - Vector2D(0.0, d), closestT)
        val y2 = x2
        val y3 = this.error(r + Vector2D(0.0, d), closestT)

        val dy1 = (y2 - y1) / d
        val dy2 = (y3 - y2) / d

        val ddy = (dy2 - dy1) / d

        val xy1 = this.error(r + Vector2D(-d/2, -d/2), closestT)
        val xy2 = this.error(r + Vector2D(d/2, -d/2), closestT)
        val xy3 = this.error(r + Vector2D(-d/2, d/2), closestT)
        val xy4 = this.error(r + Vector2D(d/2, d/2), closestT)

        val dydx = ((xy4 - xy3) / d - (xy2 - xy1) / d) / d

        return Array2DRowRealMatrix(arrayOf(doubleArrayOf(ddx, dydx), doubleArrayOf(dydx, ddy)))
    }

    override fun n_vec(r: Vector2D, closestT: Double): Vector2D {
        val pathPt = calculatePoint(closestT)
        val ptPathVec = r - pathPt
        return ptPathVec.normalized()
    }
}