package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autonomous Long", group="Autonomous mode")
// @Disabled
public class BBOTZ_AutonomousMode_Long extends LinearOpMode {

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

        // Advance 36", then stop (700)
        autonomousModeCommonMethods.driveForwardUsingODS();

        // Launch ball
        autonomousModeCommonMethods.launchBall();

        // Advance 30" to knock off big ball, then stop
        autonomousModeCommonMethods.driveForwardUsingTime(2500);

        autonomousModeCommonMethods.turnRight(800);
    }
}
