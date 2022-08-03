package org.firstinspires.ftc.teamcode

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

/**
 * A `TeleOp` for a single driver
 *
 * The drivetrain is controlled with the left and right stick.
 *
 * @see LinearOpMode
 * @see TeleOp
 */
@TeleOp(name="SamOneDriver", group="Tungsteel")
class OneDriver : LinearOpMode() {
    /** Main event loop for the OpMode. */
    override fun runOpMode() {
        val robot = Timothy(hardwareMap)
        val gamepad = GamepadEx(gamepad1)
        waitForStart()

        while (opModeIsActive()) {
            gamepad.readButtons()

            if (gamepad.wasJustPressed(GamepadKeys.Button.A)) {
                robot.shooter.fireRing()
            }

            robot.drivetrain.goXYR(
                    gamepad1.left_stick_x.toDouble(),
                    -gamepad1.left_stick_y.toDouble(),
                    gamepad1.right_stick_x.toDouble(),
            )
        }
    }
}