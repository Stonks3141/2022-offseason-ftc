package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.util.Encoder

/*
* Sample tracking wheel localizer implementation assuming the standard configuration:
*
*    ^
*    |
*    | ( x direction)
*    |
*    v
*    <----( y direction )---->
*        (forward)
*    /--------------\
*    |     ____     |
*    |     ----     |    <- Perpendicular Wheel
*    |           || |
*    |           || |    <- Parallel Wheel
*    |              |
*    |              |
*    \--------------/
*
*/
class TwoWheelTrackingLocalizer(hardwareMap: HardwareMap, private val drive: SampleMecanumDrive) :
    TwoTrackingWheelLocalizer(
        listOf(
            Pose2d(PARALLEL_X, PARALLEL_Y, 0.0),
            Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90.0))
        )
    ) {
    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private val parallelEncoder: Encoder
    private val perpendicularEncoder: Encoder
    override fun getHeading(): Double {
        return drive.rawExternalHeading
    }

    override fun getHeadingVelocity(): Double {
        return drive.getExternalHeadingVelocity()
    }

    override fun getWheelPositions(): List<Double> {
        return listOf(
            encoderTicksToInches(parallelEncoder.currentPosition.toDouble()),
            encoderTicksToInches(perpendicularEncoder.currentPosition.toDouble())
        )
    }

    override fun getWheelVelocities(): List<Double> {
        return listOf(
            encoderTicksToInches(parallelEncoder.correctedVelocity),
            encoderTicksToInches(perpendicularEncoder.correctedVelocity)
        )
    }

    companion object {
        var TICKS_PER_REV = 8192.0
        var WHEEL_RADIUS = 0.75 // in
        var GEAR_RATIO = 1.0 // output (wheel) speed / input (encoder) speed
        var PARALLEL_X = 2.92 // X is the up and down direction
        var PARALLEL_Y = 5.05 // Y is the strafe direction
        var PERPENDICULAR_X = -5.38
        var PERPENDICULAR_Y = 1.59
        fun encoderTicksToInches(ticks: Double): Double {
            return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
        }
    }

    init {
        parallelEncoder = Encoder(
            hardwareMap.get(DcMotorEx::class.java, "parallelEncoder")
        )
        perpendicularEncoder = Encoder(
            hardwareMap.get(DcMotorEx::class.java, "perpendicularEncoder")
        )

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }
}