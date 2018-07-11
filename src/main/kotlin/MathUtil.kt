import org.apache.commons.math3.linear.Array2DRowRealMatrix
import kotlin.math.E
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.pow

val MATRIX_E = Array2DRowRealMatrix(arrayOf(doubleArrayOf(0.0, 1.0), doubleArrayOf(-1.0, 0.0)))
val MATRIX_I2 = Array2DRowRealMatrix(arrayOf(doubleArrayOf(1.0, 0.0), doubleArrayOf(0.0, 1.0)))

fun lerp(a: Double, b: Double, t: Double): Double {
    return (b - a) * t + a
}

fun invLerp(a: Double, b: Double, l: Double): Double {
    return (l - a) / (b - a)
}

fun normalizeAngle(angle: Double): Double {
    var ang = angle % (2 * PI)
    if (ang < 0) {
        ang += 2 * PI
    }
    return ang
}


fun toHeading(angle: Double): Double {
    var n = normalizeAngle(angle)
    if (n > PI) {
        n -= 2 * PI
    }
    return n
}

fun sigmoid(x: Double, k: Double): Double {
    return 1.0 / (1 + E.pow(-k * x))
}

fun minimize(func: (Double) -> Double, guess: Double): Double {
    var currentGuess: Double
    var nextGuess = guess
    val d = 0.0001
    do {
        currentGuess = nextGuess
        val deriv = (func(currentGuess - d * 0.5) + func(currentGuess + d * 0.5)) / d
        nextGuess = guess - func(currentGuess) / deriv
    } while ((nextGuess - currentGuess).absoluteValue > 1.0/10000)
    return nextGuess
}