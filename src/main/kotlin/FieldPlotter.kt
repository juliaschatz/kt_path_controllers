
import org.knowm.xchart.SwingWrapper
import org.knowm.xchart.QuickChart
import org.knowm.xchart.style.markers.Marker
import org.knowm.xchart.style.markers.None
import java.awt.Color


fun main(args: Array<String>) {

    val pts = arrayOf(Pose(0.0, 0.0, 0.0), Pose(10.0, 10.0, 0.0))
    println(biarcInterpolate(pts[0], pts[1]))
    val path = EllipsePath(0.0, 0.0, 5.0, 5.0)
    val controller = GVFController(path, 10.0, 2.0)
    val xPath = DoubleArray(201)
    val yPath = DoubleArray(201)

    for (i in 0..200) {
        val t = i / 200.0
        val xy = path.calculatePoint(t)
        xPath.set(i, xy.x)
        yPath.set(i, xy.y)
    }
    val xRobot = DoubleArray(1000)
    val yRobot = DoubleArray(1000)
    xRobot[0] = 0.0
    yRobot[0] = -3.0
    var heading = Vector2D(-1.0, 0.0)
    val speed = 20.0 / 1000
    for (t in 1..999) {
        var position = Vector2D(xRobot[t-1], yRobot[t-1])
        val curvature: Double
        try {
            curvature = controller.curvatureControl(position, speed, heading)
        }
        catch (e: IllegalArgumentException) {
            continue
        }
        position += heading.scalarMul(speed)
        if (curvature == 0.0) {

        }
        else {
            val radiusRatio = {R: Double, D: Double -> (R - D/2) / (R + D/2)}
            val radius = -1.0 / curvature
            var leftSpeed: Double
            var rightSpeed: Double
            if (radius > 0) {
                leftSpeed = speed
                rightSpeed = leftSpeed * radiusRatio(radius, 1.0)
            }
            else {
                rightSpeed = speed
                leftSpeed = rightSpeed * radiusRatio(-radius, 1.0)
            }
            val w = (rightSpeed - leftSpeed) / 1.0
            heading = Vector2D.fromAngle(heading.angle() + w)
        }
        xRobot[t] = position.x
        yRobot[t] = position.y
    }

    val chart = QuickChart.getChart("Path", "X", "Y", "Path", xPath, yPath)
    val robotSeries = chart.addSeries("Robot Position",  xRobot, yRobot)
    robotSeries.marker = None()
    robotSeries.lineColor = Color.ORANGE
// Show it
    SwingWrapper(chart).displayChart()



}