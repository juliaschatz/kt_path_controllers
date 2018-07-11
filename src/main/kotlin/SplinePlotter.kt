import org.knowm.xchart.QuickChart
import org.knowm.xchart.SwingWrapper
import org.knowm.xchart.style.markers.None
import java.awt.Color

fun main(args: Array<String>) {
    val arc = Spline(arrayOf(Pose(0.0, 0.0, 0.0), Pose(2.0, 2.0, 0.0)))

    val xPath = DoubleArray(201)
    val yPath = DoubleArray(201)
    for (i in 0..200) {
        val t = i / 200.0
        val xy = arc.calculatePoint(t)
        xPath[i] = xy.x
        yPath[i] = xy.y
    }

    val xAppr = ArrayList<Double>()
    val yAppr = ArrayList<Double>()
    arc.polynomials.forEach { it.arcs.forEach {
        for (i in 0..100) {
            val pt = it.wrapped.r(i / 100.0)
            xAppr.add(pt.x)
            yAppr.add(pt.y)
        }

    } }


    val chart = QuickChart.getChart("Path", "X", "Y", "Path", xPath, yPath)
    val appr = chart.addSeries("Approximation", xAppr, yAppr)
    appr.marker = None()
    appr.lineColor = Color.ORANGE
    SwingWrapper(chart).displayChart()
}