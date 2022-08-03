package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import kotlin.math.sin

/**
 * The Mecanum drivetrain for the robot.
 *
 * # Example
 *
 * ```kotlin
 * val drivetrain = Drivetrain(hwMap)
 * drivetrain.goXYR(1.0, 2.0, 3.0)
 * ```
 *
 * @constructor takes a `HardwareMap` and extracts the motors
 * @property speedMod the factor that motor power is multiplied by
 *
 * @see HardwareMap
 */
class Drivetrain(hwMap: HardwareMap) {
    var speedMod: Double = 0.8

    private val leftFront = hwMap.get(DcMotorEx::class.java, "leftFront")
    private val leftRear = hwMap.get(DcMotorEx::class.java, "leftRear")
    private val rightFront = hwMap.get(DcMotorEx::class.java, "rightFront")
    private val rightRear = hwMap.get(DcMotorEx::class.java, "rightRear")

    init {
        leftFront.direction = DcMotorSimple.Direction.FORWARD
        leftRear.direction = DcMotorSimple.Direction.FORWARD
        rightFront.direction = DcMotorSimple.Direction.REVERSE
        rightRear.direction = DcMotorSimple.Direction.REVERSE
    }

    /**
     * Moves the robot along the specified (`x`, `y`) vector and rotates it to `r`.
     *
     * @param x x component of velocity vector
     * @param y y component of velocity vector
     * @param r direction to face
     */
    fun goXYR(x: Double, y: Double, r: Double) {
        val lfPower = (y + x + r) * speedMod
        val lbPower = (y - x + r) * speedMod
        val rfPower = (y - x - r) * speedMod
        val rbPower = (y + x - r) * speedMod

        leftFront.power = Range.clip(lfPower, -1.0, 1.0)
        leftRear.power = Range.clip(lbPower, -1.0, 1.0)
        rightFront.power = Range.clip(rfPower, -1.0, 1.0)
        rightRear.power = Range.clip(rbPower, -1.0, 1.0)
    }

    /**
     * Moves the robot at an `angle` by a `magnitude`.
     *
     * @param angle the angle to move along
     * @param magnitude how far to move
     */
    fun goPolarRadians(angle: Double, magnitude: Double) { // Strafes the robot at a specified angle
        val piOver4 = Math.PI / 4 //see Seamonster's mecanum site for math
        val leftSlant = sin(angle - piOver4) * Range.clip(magnitude, -1.0, 1.0)
        val rightSlant = sin(angle + piOver4) * Range.clip(magnitude, -1.0, 1.0)
        leftFront.power = leftSlant
        leftRear.power = leftSlant
        rightFront.power = rightSlant
        rightRear.power = rightSlant
    }
}