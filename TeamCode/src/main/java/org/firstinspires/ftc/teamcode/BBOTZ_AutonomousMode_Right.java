//AUTONOUMOUS MODE RIGHT
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Autonomous Backup Right", group="Autonomous mode")
// @Disabled
public class BBOTZ_AutonomousMode_Right extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        BBOTZ_AutonomousMode_Common_Methods autonomousModeCommonMethods = new BBOTZ_AutonomousMode_Common_Methods(hardwareMap);

        // reset encoder
        autonomousModeCommonMethods.resetEncoder();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Advance 36", then stop
        autonomousModeCommonMethods.driveForward(0);

        // Launch ball
        autonomousModeCommonMethods.launchBall();

        //Turn to knock off ball
        autonomousModeCommonMethods.tankTurnRight(300);

        // Advance 30" to go past cap ball
        autonomousModeCommonMethods.driveForward(2700);

        // Turn to knock off ball
        // Note: the value does not match the mirror autonomous mode because
        // the strengh of the motors are not equal.
        autonomousModeCommonMethods.tankTurnLeft(1200);

        // Backup up ramp ~70"
        autonomousModeCommonMethods.driveBackward(8000);
    }
}
