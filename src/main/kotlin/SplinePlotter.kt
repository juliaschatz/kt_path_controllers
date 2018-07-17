import org.knowm.xchart.QuickChart
import org.knowm.xchart.SwingWrapper
import org.knowm.xchart.style.markers.None
import java.awt.Color
import kotlin.math.PI

fun main(args: Array<String>) {
    val arc = Spline(arrayOf(
            Pose(0.0, 0.0, PI / 2),
            Pose(2.0, 2.0, 0.0)))

    val xPath = DoubleArray(201)
    val yPath = DoubleArray(201)
    val ts = DoubleArray(201)
    val curv = DoubleArray(201)
    //val curvAppr = DoubleArray(201)
    for (i in 0..200) {
        val t = i / 200.0
        val xy = arc.calculatePoint(t)
        xPath[i] = xy.x
        yPath[i] = xy.y
        ts[i] = t
        curv[i] = arc.curvature(t)
    }

    val xAppr = ArrayList<Double>()
    val yAppr = ArrayList<Double>()
    var arcCt = 0
    arc.polynomials.forEach { it.arcs.forEach {
        arcCt++
        for (i in 0..100) {
            val pt = it.wrapped.r(i / 100.0)
            xAppr.add(pt.x)
            yAppr.add(pt.y)
        }

    } }
    println("$arcCt arcs")
    val chart = QuickChart.getChart("Path", "X", "Y", "Path", xPath, yPath)
    val appr = chart.addSeries("Approximation", xAppr, yAppr)
    appr.marker = None()
    appr.lineColor = Color.ORANGE

    val curvChart = QuickChart.getChart("Curvature", "T", "Curvature", "Curvature", ts, curv)
    SwingWrapper(curvChart).displayChart()

    arrayOf(Vector2D(2.0, 2.0),
            Vector2D(1.0, 1.0),
            Vector2D(2.5, 3.0),
            Vector2D(0.5, 0.5),
            Vector2D(1.0, 2.0)).forEach {
        //val clsPt = arc.calculatePoint(arc.closestTOnPathToArcs(it))
        //chart.addSeries("Cls $it", doubleArrayOf(it.x, clsPt.x), doubleArrayOf(it.y, clsPt.y))
    }



    SwingWrapper(chart).displayChart()
}