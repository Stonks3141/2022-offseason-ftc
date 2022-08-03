package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import java.util.*
import kotlin.concurrent.schedule

private const val EXTENDED = 1.0
private const val RETRACTED = 0.0

class Shooter(hwMap: HardwareMap) {
    private val bumper = hwMap.get(Servo::class.java, "bumper")
    private val spinner = hwMap.get(DcMotorEx::class.java, "spinner")

    init {
        bumper.direction = Servo.Direction.FORWARD
        bumper.position = RETRACTED
        spinner.direction = DcMotorSimple.Direction.FORWARD
    }

    fun fireRing() {
        spinner.power = 1.0
        bumper.position = EXTENDED
        Timer("retracting", false).schedule(500) {
            bumper.position = RETRACTED
            spinner.power = 0.0
        }
    }
}