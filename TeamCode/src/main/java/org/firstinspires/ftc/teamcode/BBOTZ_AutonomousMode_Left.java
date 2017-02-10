//AUTONOMOUS MODE LEFT
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Autonomous Backup Left", group="Autonomous mode")
// @Disabled
public class BBOTZ_AutonomousMode_Left extends LinearOpMode {

    BBOTZ_AutonomousMode_Common_Methods autonomousModeCommonMethods;

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        autonomousModeCommonMethods = new BBOTZ_AutonomousMode_Common_Methods(hardwareMap);

        // reset encoder
        autonomousModeCommonMethods.resetEncoder();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Advance 36", then stop
        autonomousModeCommonMethods.driveForwardUsingTime(0);

        // Launch ball
        autonomousModeCommonMethods.launchBall();

        // Turn to knock off ball
        autonomousModeCommonMethods.tankTurnLeft(300);

        // Advance 30" to go past cap ball
        autonomousModeCommonMethods.driveForwardUsingTime(2700);

        // Turn to knock off ball
        autonomousModeCommonMethods.tankTurnRight(1500);

        // Backup up ramp ~70"
        autonomousModeCommonMethods.driveBackwardUsingTime(8000);
    }
}
