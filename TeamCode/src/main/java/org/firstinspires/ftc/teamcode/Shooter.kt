package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import java.util.Timer
import kotlin.concurrent.schedule

private const val RETRACTED = 0.8
private const val EXTENDED = 0.3

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