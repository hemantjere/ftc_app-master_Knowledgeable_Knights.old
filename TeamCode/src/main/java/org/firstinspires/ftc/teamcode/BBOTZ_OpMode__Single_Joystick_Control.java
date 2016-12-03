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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
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

@TeleOp(name="Template: Single Joystick Control OpMode", group="OpMode Control")  // @Autonomous(...) is the other common choice
// @Disabled
public class BBOTZ_OpMode__Single_Joystick_Control extends LinearOpMode {

    private double SPIN_ARM_FORWARD_MAXSPEED = 1d;
    private double SPIN_ARM_REVERSE_MAXSPEED = .7d;
    private double SPIN_ARM_MINSPEED = .3d;
    private double SPIN_ARM_STOP = 0d;
    private double ZIPTIE_MOTOR_SPEED = 1d;
    private double ZIPTIE_MOTOR_STOP = 0d;
    private double DEADZONE = .1d;
    private long SPIN_ARM_ROTATE_TIME = 200;

    private float DRIVE_MULTIPLE = .2f;

    private double HAND_SPEED = .01d;

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftDrive = null;
    DcMotor rightDrive = null;
    DcMotor spinArm = null;
    Servo leftArm = null;
    Servo rightArm = null;
    Servo leftHand = null;
    Servo rightHand = null;
    DcMotor ziptieMotor = null;

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
        leftHand = hardwareMap.servo.get("left hand");
        rightHand = hardwareMap.servo.get("right hand");
        ziptieMotor = hardwareMap.dcMotor.get("ziptie motor");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        spinArm.setDirection(DcMotor.Direction.FORWARD);
        ziptieMotor.setDirection(DcMotor.Direction.REVERSE);

        Boolean toggleYButtonZiptie = false;
        Boolean toggleYButtonInverse = false;
        
        int motorCount = 0;
        float leftAccelerationRate = 0.0f;
        float rightAccelerationRate = 0.0f;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // get direction does not display unless you set the direction
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Status", "gamepad1: " + gamepad1.toString());
            telemetry.update();

            //One Stick Control
//            if(gamepad1.right_stick_y != 0){
//                //forward and backward
//                leftDrive.setPower(gamepad1.right_stick_y);
//                rightDrive.setPower(gamepad1.right_stick_y);
//            }
//
//            if(gamepad1.right_stick_x != 0){
//                //rotating (tank turn)
//                leftDrive.setPower(gamepad1.right_stick_x);
//                rightDrive.setPower(-1 * gamepad1.right_stick_x);
//            }
//
//            if(gamepad1.right_stick_y == 0 && gamepad1.right_stick_x == 0){
//                leftDrive.setPower(0);
//                rightDrive.setPower(0);
//            }
            if (gamepad1.left_bumper == true) {
                leftDrive.setPower((gamepad1.right_stick_y - gamepad1.right_stick_x)*DRIVE_MULTIPLE);
                rightDrive.setPower((gamepad1.right_stick_y + gamepad1.right_stick_x)*DRIVE_MULTIPLE);
            }
            else{
                leftDrive.setPower(gamepad1.right_stick_y - gamepad1.right_stick_x);
                rightDrive.setPower(gamepad1.right_stick_y + gamepad1.right_stick_x);
            }
            //Beacon Press
            if (gamepad1.dpad_up == true) {
                // forward for beacon
                leftDrive.setPower(-1);
                rightDrive.setPower(-1);
                sleep(1000);
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }

            if (gamepad1.dpad_down == true) {
                // backward for beacon
                leftDrive.setPower(1);
                rightDrive.setPower(1);
                sleep(1000);
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }


            //Inverse Motors : Decided not to do with one stick
//                leftDrive.setPower(gamepad1.left_stick_y*DRIVE_MULTIPLE);
//                rightDrive.setPower(gamepad1.right_stick_y*DRIVE_MULTIPLE);
//            }
//            else if(motorCount == 1) {
//                // accelerate robot slowly (avoid slippage)
//                leftDrive.setPower(gamepad1.left_stick_y);
//                rightDrive.setPower(gamepad1.right_stick_y);
//                sleep(250);
//            }
//            else {
//                leftDrive.setPower(gamepad1.left_stick_y);
//                rightDrive.setPower(gamepad1.right_stick_y);
//            }
//
//            if ((gamepad1.left_stick_y == 0) && gamepad1.right_stick_y == 0){
//                motorCount=0;
//            }
//            if((gamepad1.left_stick_y != 0) || (gamepad1.right_stick_y != 0)){
//                motorCount++;
//            }

            // Spin Arm setup
            if(gamepad2.right_trigger> DEADZONE){
                // slow throw mode
                spinArm.setDirection(DcMotor.Direction.REVERSE);
                spinArm.setPower(gamepad2.right_trigger*SPIN_ARM_MINSPEED);
            }
            else if(gamepad2.right_bumper==true){
                // fast throw mode
                spinArm.setDirection(DcMotor.Direction.REVERSE);
                spinArm.setPower(SPIN_ARM_FORWARD_MAXSPEED);
                sleep(SPIN_ARM_ROTATE_TIME);
                spinArm.setPower(SPIN_ARM_STOP);
            }
            else if(gamepad2.left_trigger> DEADZONE){
                // slow push mode
                spinArm.setDirection(DcMotor.Direction.FORWARD);
                spinArm.setPower(gamepad2.left_trigger*SPIN_ARM_MINSPEED);
            }
            else if(gamepad2.left_bumper==true){
                // fast push mode
                spinArm.setDirection(DcMotor.Direction.FORWARD);
                spinArm.setPower(SPIN_ARM_REVERSE_MAXSPEED);
                sleep(SPIN_ARM_ROTATE_TIME);
                spinArm.setPower(SPIN_ARM_STOP);
            }
            else {
                spinArm.setPower(SPIN_ARM_STOP);
            }

            //Ziptie Motor setup
            if(gamepad2.y==true){
                toggleYButtonZiptie = !toggleYButtonZiptie;
                sleep(500);
            }

            if (toggleYButtonZiptie==true) {
                ziptieMotor.setPower(ZIPTIE_MOTOR_SPEED);
            }
            else {
                ziptieMotor.setPower(ZIPTIE_MOTOR_STOP);
            }

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
