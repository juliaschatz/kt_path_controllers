import org.knowm.xchart.QuickChart
import org.knowm.xchart.SwingWrapper

fun main(args: Array<String>) {
    val arc = Biarc.ArcSegment.fromThreePoints(Vector2D(0.0, 0.0), Vector2D(5.0, 0.0), Vector2D(5.0, 5.0))

    val xPath = DoubleArray(201)
    val yPath = DoubleArray(201)
    for (i in 0..200) {
        val t = i / 200.0
        val xy = arc.r(t)
        xPath.set(i, xy.x)
        yPath.set(i, xy.y)
    }

    val chart = QuickChart.getChart("paths.Path", "X", "Y", "paths.Path", xPath, yPath)
    SwingWrapper(chart).displayChart()
}