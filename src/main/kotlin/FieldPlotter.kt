
import org.knowm.xchart.SwingWrapper
import org.knowm.xchart.QuickChart
import org.knowm.xchart.style.markers.None
import java.awt.Color
import kotlin.math.PI
import kotlin.math.acos


fun main(args: Array<String>) {

    val pts = arrayOf(Pose(0.0, 0.0, 0.0), Pose(10.0, 10.0, 0.0))
    println(biarcInterpolate(pts[0], pts[1]))
    val path: Path
    val ellipse = true
    val curvatureControl = true
    if (ellipse) {
        path = EllipsePath(0.0, 0.0, 5.0, 5.0)
    }
    else {
        path = BiarcPath(pts)
    }
    println(path.tangentVec(0.5))
    val hVec = Vector2D(5.0, 5.0)
    println(path.hessian(hVec, path.closestTOnPathTo(hVec)))
    val controller = GVFController(path, 1.0, 1.0)
    val xPath = DoubleArray(201)
    val yPath = DoubleArray(201)
    val err = DoubleArray(1000)
    val ts = DoubleArray(1000)
    val desAngDiff = DoubleArray(1000)
    val actualCurv = DoubleArray(1000)

    for (i in 0..200) {
        val t = i / 200.0
        val xy = path.calculatePoint(t)
        xPath.set(i, xy.x)
        yPath.set(i, xy.y)
    }
    val xRobot = DoubleArray(1000)
    val yRobot = DoubleArray(1000)
    xRobot[0] = 5.0
    yRobot[0] = 0.0
    var heading = Vector2D.fromAngle(PI / 2)
    val speed = 40.0 / 1000
    for (t in 1..999) {
        var position = Vector2D(xRobot[t-1], yRobot[t-1])
        var curvature: Double
        try {
            if (curvatureControl) {
                curvature = controller.curvatureControl(position, speed, heading)
                actualCurv[t] = curvature
                desAngDiff[t] = acos(heading.dot(controller.vectorControl(position).normalized()))
            }
            else {
                heading = controller.vectorControl(position)
                curvature = 0.0
            }

            err[t-1] = controller.path.levelSet(position, controller.path.closestTOnPathTo(position))
            ts[t-1] = t.toDouble()
        }
        catch (e: IllegalArgumentException) {
            //position += heading.scalarMul(speed)
            xRobot[t] = xRobot[t-1]
            yRobot[t] = yRobot[t-1]
            continue
        }
        position += heading.scalarMul(speed)
        if (curvature == 0.0) {

        }
        else {
            val radiusRatio: (Double, Double) -> Double = {R: Double, D: Double -> (R - D/2.0) / (R + D/2.0)}
            val radius = 1.0 / curvature
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

    SwingWrapper(QuickChart.getChart("Error", "T", "Error", "Error", ts, err)).displayChart()
    SwingWrapper(QuickChart.getChart("Curvature", "T", "Curvature", "Curvature", ts, actualCurv)).displayChart()
    SwingWrapper(QuickChart.getChart("Desired Angle Difference", "T", "Angle to Desired", "Angle", ts, desAngDiff)).displayChart()



}