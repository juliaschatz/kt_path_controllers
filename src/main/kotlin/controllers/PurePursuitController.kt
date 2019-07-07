package controllers

import math.Vector2D
import paths.Path
import math.Pose
import kotlin.math.pow

class PurePursuitController(path: Path, val lookaheadBase: Double): PathController(path) {
    private var lastT = 0.001
    fun getLookaheadPoint(t: Double, speed: Double, pose: Pose): Vector2D {
        val lookahead = lookaheadBase
        return path.calculatePoint(t + lookahead / path.length())
    }

    override fun curvatureControl(pose: Pose, speed: Double, dt: Double): Double {
        val closestT = path.closestTOnPathTo(pose, lastT)
        lastT = closestT
        val target = pose.translate(getLookaheadPoint(closestT, speed, pose))
        return -2 * target.y / target.norm().pow(2)
    }

    override fun vectorControl(pose: Pose, speed: Double, dt: Double): Vector2D {
        val closestT = path.closestTOnPathTo(pose, lastT)
        lastT = closestT
        val target = pose.translate(getLookaheadPoint(closestT, speed, pose))
        return target.normalized()
    }
}