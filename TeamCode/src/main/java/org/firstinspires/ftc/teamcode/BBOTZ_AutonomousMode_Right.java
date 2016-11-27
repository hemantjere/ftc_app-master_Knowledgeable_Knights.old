/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Autonomous Backup Right", group="Autonomous mode")
// @Disabled
public class BBOTZ_AutonomousMode_Right extends LinearOpMode {

    private double MAX_DRIVE_SPEED = .2d;
    private double STOP_DRIVE = 0d;
    private double SPIN_ARM_FORWARD_MAXSPEED = 1d;
    private double SPIN_ARM_STOP = 0d;
    private long SPIN_ARM_ROTATE_TIME = 800;

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftDrive = null;
    DcMotor rightDrive = null;
    DcMotor spinArm = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftDrive = hardwareMap.dcMotor.get("left drive wheel");
        rightDrive = hardwareMap.dcMotor.get("right drive wheel");
        spinArm = hardwareMap.dcMotor.get("spin arm");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        spinArm.setDirection(DcMotor.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Advance 36", then stop
        leftDrive.setPower(MAX_DRIVE_SPEED);
        rightDrive.setPower(MAX_DRIVE_SPEED);
        Thread.sleep(1900);
        leftDrive.setPower(STOP_DRIVE);
        rightDrive.setPower(STOP_DRIVE);
        Thread.sleep(1000);

        // Launch ball
        spinArm.setPower(SPIN_ARM_FORWARD_MAXSPEED);
        Thread.sleep(SPIN_ARM_ROTATE_TIME);
        spinArm.setPower(SPIN_ARM_STOP);
        Thread.sleep(1000);

        // Advance 30" to knock off big ball, then stop
        leftDrive.setPower(MAX_DRIVE_SPEED);
        rightDrive.setPower(MAX_DRIVE_SPEED);
        Thread.sleep(1400);
        leftDrive.setPower(STOP_DRIVE);
        rightDrive.setPower(STOP_DRIVE);
        Thread.sleep(1000);

        // Rotate left ~40 degrees
        rightDrive.setPower(MAX_DRIVE_SPEED);
        Thread.sleep(750);
        rightDrive.setPower(STOP_DRIVE);
        Thread.sleep(1000);

        // Backup up ramp ~70"
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setPower(MAX_DRIVE_SPEED);
        rightDrive.setPower(MAX_DRIVE_SPEED);
        Thread.sleep(5000);
        leftDrive.setPower(STOP_DRIVE);
        rightDrive.setPower(STOP_DRIVE);
    }
}
