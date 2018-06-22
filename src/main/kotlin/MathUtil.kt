import org.apache.commons.math3.linear.Array2DRowRealMatrix

val MATRIX_E = Array2DRowRealMatrix(arrayOf(doubleArrayOf(0.0, 1.0), doubleArrayOf(-1.0, 0.0)))
val MATRIX_I2 = Array2DRowRealMatrix(arrayOf(doubleArrayOf(1.0, 0.0), doubleArrayOf(0.0, 1.0)))

fun lerp(a: Double, b: Double, t: Double): Double {
    return (b - a) * t + a
}