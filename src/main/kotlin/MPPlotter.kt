import motionprofile.MotionProfile
import motionprofile.TrapezoidalMotionProfile
import org.knowm.xchart.QuickChart
import org.knowm.xchart.SwingWrapper
import org.knowm.xchart.style.markers.None
import paths.SplinePath
import java.awt.Color
import kotlin.math.PI

fun main(args: Array<String>) {
    val count = 100
    val mp = TrapezoidalMotionProfile(10.0, MotionProfile.Limits(3.0, 3.0, 0.0))
    val position = DoubleArray(count+1)
    val speed = DoubleArray(count+1)
    val times = DoubleArray(count+1)
    for (i in 0..count) {
        val t = mp.getTime() * i / count
        position[i] = mp.getPosition(t)
        speed[i] = mp.getSpeed(t)
        times[i] = t
    }
    val chart = QuickChart.getChart("Motion Profile", "Time", "", arrayOf("Position", "Speed"), times, arrayOf(position, speed))
    SwingWrapper(chart).displayChart()
}