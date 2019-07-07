package math

import org.apache.commons.math3.linear.*
import kotlin.math.*

open class Vector2D(open val x: Double, open val y: Double) {
    companion object {
        fun fromApacheVec(v: RealVector): Vector2D {
            return Vector2D(v.getEntry(0), v.getEntry(1))
        }

        fun fromAngle(angle: Double): Vector2D {
            return Vector2D(cos(angle), sin(angle))
        }

        fun arePointsCollinear(r1: Vector2D, r2: Vector2D, r3: Vector2D): Boolean {
            val denomMat = Array2DRowRealMatrix(arrayOf(
                    doubleArrayOf(r1.x, r1.y, 1.0),
                    doubleArrayOf(r2.x, r2.y, 1.0),
                    doubleArrayOf(r3.x, r3.y, 1.0)
            ))
            return LUDecomposition(denomMat).determinant == 0.0
        }
    }
    fun dot(other: Vector2D): Double {
        return other.x * this.x + other.y * this.y
    }
    fun norm(): Double {
        return this.sqNorm().pow(0.5)
    }
    fun sqNorm(): Double {
        return x.pow(2) + y.pow(2)
    }
    fun normalized(): Vector2D {
        val len = this.norm()
        return Vector2D(x / len, y / len)
    }
    fun getRightNormal(): Vector2D {
        return Vector2D(y, -x)
    }
    fun getLeftNormal(): Vector2D {
        return Vector2D(-y, x)
    }
    fun toApacheVec(): RealVector {
        return ArrayRealVector(doubleArrayOf(this.x, this.y))
    }
    fun outerProduct(): RealMatrix {
        return Array2DRowRealMatrix(arrayOf(doubleArrayOf(x * x, x * y), doubleArrayOf(x * y, y * y)))
    }
    fun scalarMul(d: Double): Vector2D {
        return Vector2D(x * d, y * d)
    }
    fun angle(): Double {
        return atan2(y, x)
    }
    fun zProd(other: Vector2D): Double {
        return this.x * other.y - this.y * other.x
    }

    operator fun minus(other: Vector2D): Vector2D {
        return Vector2D(this.x - other.x, this.y - other.y)
    }

    operator fun plus(other: Vector2D): Vector2D {
        return Vector2D(this.x + other.x, this.y + other.y)
    }
    override fun equals(other: Any?): Boolean {
        if (other is Vector2D) {
            return other.x == x && other.y == y
        }
        return false
    }

    override fun hashCode(): Int {
        var result = x.hashCode()
        result = 31 * result + y.hashCode()
        return result
    }

    override fun toString(): String {
        return "math.Vector2D($x, $y)"
    }

    fun sqDist(other: Vector2D): Double {
        return (other.x - x).pow(2) + (other.y - y).pow(2)
    }
    fun dist(other: Vector2D): Double {
        return sqDist(other).pow(0.5)
    }

    fun neg(): Vector2D {
        return this.scalarMul(-1.0)
    }

    fun divide(other: Vector2D): Double {
        return norm() / other.norm()
    }
}