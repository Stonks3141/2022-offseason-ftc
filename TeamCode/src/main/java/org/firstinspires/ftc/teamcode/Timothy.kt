package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.HardwareMap
import java.util.Timer
import kotlin.concurrent.schedule

/**
 * The class that controls the hardware.
 *
 * @constructor takes a `hardwareMap` and passes it to subsystems
 * @property drivetrain the robot's drivetrain
 * @property shooter the ring-shooting mechanism
 *
 * @see Drivetrain
 * @see Shooter
 * @see HardwareMap
 */
class Timothy(hwMap: HardwareMap) {
    val drivetrain = Drivetrain(hwMap)
    val shooter = Shooter(hwMap)
}