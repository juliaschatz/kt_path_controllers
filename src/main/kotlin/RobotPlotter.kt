import controllers.TimeVariantPurePursuitController
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
    val CURVATURE = true

    val path = SplinePath(arrayOf(
            Pose(0.0, 0.0, 0.0),
            Pose(20.0, 0.0, -PI / 4)))
    println(path.length)
    //val controller = TimeVariantGVFController(path, 10.0, 0.4, MotionProfile.Limits(6.0, 3.0, 0.0))
    val controller = TimeVariantPurePursuitController(path, 3.0, MotionProfile.Limits(6.0, 3.0, 0.0))
    val dt = 10.0 / 1000.0

    val xRobot = ArrayList<Double>()
    val yRobot = ArrayList<Double>()
    val times = ArrayList<Double>()
    val errs = ArrayList<Double>()

    var time = 0.0
    var position = Vector2D(0.00, -0.0)
    var heading = Vector2D.fromAngle(0.0 * PI / 2)

    while (true) {
        if (time >= 10.0) {
            break
        }
        var curvature: Double
        var speed: Double
        if (CURVATURE) {
            try {
                val pose = Pose(position.x, position.y, heading.angle())
                val command = controller.curvatureControl(pose, time)
                speed = command.speed
                curvature = command.curvature
            } catch (e: IllegalArgumentException) {
                println("controller error")
                break
            }

            if (curvature == 0.0) {

            } else {
                val radiusRatio: (Double, Double) -> Double = { R: Double, D: Double -> (R - D / 2.0) / (R + D / 2.0) }
                val radius = 1.0 / curvature
                var leftSpeed: Double
                var rightSpeed: Double
                val D = 1.0
                if (radius > 0) {
                    leftSpeed = speed
                    rightSpeed = leftSpeed * radiusRatio(radius, D)
                } else {
                    rightSpeed = speed
                    leftSpeed = rightSpeed * radiusRatio(-radius, D)
                }
                val w: Double = (rightSpeed - leftSpeed) / D
                heading = Vector2D.fromAngle(heading.angle() + w * dt)
            }
            position += heading.scalarMul(speed * dt)
        }
        else {
            val pose = Pose(position.x, position.y, heading.angle())
            heading = controller.vectorControl(pose, time)
            position += heading.scalarMul(dt)
        }


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