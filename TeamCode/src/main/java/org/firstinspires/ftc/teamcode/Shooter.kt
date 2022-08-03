package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.concurrent.schedule
import java.util.Timer

private const val RETRACTED = 0.8
private const val EXTENDED = 0.0

/**
 * The ring-shooting mechanism.
 *
 * @constructor takes a `HardwareMap` and uses the "bumper" servo and "spinner" DC motor
 *
 * @see HardwareMap
 */
class Shooter(hwMap: HardwareMap) {
    private val bumper = hwMap.get(Servo::class.java, "bumper")
    private val spinner = hwMap.get(DcMotorEx::class.java, "spinner")

    init {
        bumper.direction = Servo.Direction.FORWARD
        bumper.position = RETRACTED
        spinner.direction = DcMotorSimple.Direction.FORWARD
    }

    /**
     * Fires a ring when called.
     *
     * This function uses a scheduled callback to retract the servo and disable the spinner.
     *
     * @see kotlin.concurrent.schedule
     */
    fun fireRing() {
        spinner.power = 1.0
        bumper.position = EXTENDED
        Timer("retracting", false).schedule(1000) {
            bumper.position = RETRACTED
            spinner.power = 0.0
        }
    }
}