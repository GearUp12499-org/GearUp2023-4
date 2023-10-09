package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.odo.Pose
import org.firstinspires.ftc.teamcode.odo.degrees
import org.firstinspires.ftc.teamcode.odo.inches
import org.firstinspires.ftc.teamcode.utility.Vector2
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

private const val DISTANCE_FROM_FRONT = (142.0-10.75)
val tagPositions = mapOf(
    1 to Vector2(29.5, DISTANCE_FROM_FRONT),
    3 to Vector2(41.5, DISTANCE_FROM_FRONT),
    2 to Vector2(35.5, DISTANCE_FROM_FRONT),
)

fun detectPairToPose(first: AprilTagDetection, second: AprilTagDetection): Pose {
    val kA = first.ftcPose.range
    val kB = second.ftcPose.range
    val firstPosition = tagPositions[first.id] ?: throw IllegalArgumentException("where is tag#${first.id}")
    val secondPosition = tagPositions[second.id] ?: throw IllegalArgumentException("where is tag#${second.id}")
    val avgPose = firstPosition.add(secondPosition).scale(0.5)

    // d = half distance between the two tags
    val twoD = abs(firstPosition.x - secondPosition.x)
    val d = twoD / 2.0

    // derived solution to X with pythagorean theorem
    val x = (kA.pow(2.0) - kB.pow(2.0)) / (4 * d)
    // Ka^2 - (x + d)^2 = Y^2
    // solve for Y by plugging back into Pythagorean Theorem
    val y1 = sqrt(kA.pow(2.0) - (x + d).pow(2.0))
    // Other solution (same answer, different variables)
    // val y4 = sqrt(kB.pow(2.0) - (x - d).pow(2.0))
    // add to the midpoint between the two tags (tag 1 + D or tag 2 - D)

    return Pose((x+avgPose.x).inches, (avgPose.y-y1).inches, ((first.ftcPose.yaw + second.ftcPose.yaw) / -2.0 + 90.0).degrees)
}

fun detectSingleToPose(detection: AprilTagDetection): Pose {
    val k = detection.ftcPose.range
    val phi: Double = -detection.ftcPose.yaw.degrees.to.radians.value
    // the yaw happens to be the angle at the camera
    // such that the two legs of a right triangle w/ phi are aligned with the tag
    val globalX = k * sin(phi) // would be Y rel. to camera
    val globalY = k * cos(phi) // would be X rel. to camera
    val tagPos = tagPositions[detection.id]
        ?: throw java.lang.IllegalArgumentException("where is tag#${detection.id}")
    // offset the relative position with the known pos of the tag
    val globalAll = tagPos.add(Vector2(globalX, -globalY))
    return Pose(globalAll.x.inches, globalAll.y.inches, (detection.ftcPose.yaw + 90.0).degrees)
}