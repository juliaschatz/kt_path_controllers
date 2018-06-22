import org.apache.commons.math3.linear.*
import kotlin.math.atan2
import kotlin.math.pow

open class Vector2D(open val x: Double, open val y: Double) {
    companion object {
        fun fromApacheVec(v: RealVector): Vector2D {
            return Vector2D(v.getEntry(0), v.getEntry(1))
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
    fun toApacheVec(): RealVector {
        return ArrayRealVector(doubleArrayOf(this.x, this.y))
    }
    fun postMul(mat: RealMatrix): Vector2D {
        if (mat.isSquare && mat.rowDimension == 2) {
            val apache = mat.operate(this.toApacheVec())
            return Vector2D(apache.getEntry(0), apache.getEntry(1))
        }
        throw MatrixDimensionMismatchException(mat.rowDimension, mat.columnDimension, 2, 2)
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

    operator fun minus(other: Vector2D): Vector2D {
        return Vector2D(this.x - other.x, this.y - other.y)
    }

    operator fun plus(other: Vector2D): Vector2D {
        return Vector2D(this.x + other.x, this.y + other.y)
    }
}