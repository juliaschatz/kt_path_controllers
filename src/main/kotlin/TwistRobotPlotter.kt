import controllers.*
import math.Pose
import math.Vector2D
import motionprofile.MotionProfile
import org.knowm.xchart.QuickChart
import org.knowm.xchart.SwingWrapper
import org.knowm.xchart.style.markers.None
import paths.SplinePath
import java.awt.Color
import kotlin.math.PI

fun main(args: Array<String>) {

    val path = SplinePath(arrayOf(
            Pose(0.0, 0.0, 0.0),
            Pose(5.0, 5.0, PI / 2),
            Pose(10.0, 10.0, 0.0)))
    println(path.length)
    val controller = RamseteController(path, 2.0, 0.7, MotionProfile.Limits(6.0, 3.0, 0.0))
    val dt = 10.0 / 1000.0

    val xRobot = ArrayList<Double>()
    val yRobot = ArrayList<Double>()
    val times = ArrayList<Double>()
    val errs = ArrayList<Double>()

    var time = 0.0
    var position = Vector2D(2.00, -1.0)
    var heading = Vector2D.fromAngle(0.0 * PI / 2)

    while (true) {
        if (time >= 10.0) {
            break
        }
        var speed: Double
        val twist = controller.twistControl(Pose(position.x, position.y, heading.angle()), time)
        val w = twist.angVel
        speed = twist.linVel
        heading = Vector2D.fromAngle(heading.angle() + w * dt)
        position += heading.scalarMul(speed * dt)

        try {
            errs.add(path.error(path.levelSet(position, path.closestTOnPathTo(position, 0.001))))
        }
        catch (e: IllegalArgumentException) {
            break
        }
        xRobot.add(position.x)
        yRobot.add(position.y)
        times.add(time)
        time += dt
    }

    val xPath = DoubleArray(201)
    val yPath = DoubleArray(201)
    for (i in 0..200) {
        val t = i / 200.0
        val xy = path.calculatePoint(t)
        xPath.set(i, xy.x)
        yPath.set(i, xy.y)
    }
    println("$time")

    val chart = QuickChart.getChart("paths.Path", "X", "Y", "paths.Path", xPath, yPath)
    val robotSeries = chart.addSeries("Robot Position",  xRobot, yRobot)
    robotSeries.marker = None()
    robotSeries.lineColor = Color.ORANGE

    val errChart = QuickChart.getChart("Error", "T", "Error", "Error", times, errs)

    SwingWrapper(chart).displayChart()
    SwingWrapper(errChart).displayChart()
}