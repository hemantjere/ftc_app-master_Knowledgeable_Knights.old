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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@TeleOp(name="Template: Control OpMode", group="OpMode Control")  // @Autonomous(...) is the other common choice
// @Disabled
public class BBOTZ_OpMode_Control extends LinearOpMode {

    private double SPIN_ARM_MAXSPEED = 1d;
    private double SPIN_ARM_MINSPEED = .15d;
    private double SPIN_ARM_STOP = 0d;
    private double DEADZONE = .1d;
    private long SPIN_ARM_ROTATE_TIME = 100;

    private double DRIVE_MULTIPLE = .2d;

    private double HAND_SPEED = .05d;

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
     DcMotor leftDrive = null;
     DcMotor rightDrive = null;
     DcMotor spinArm = null;
     Servo leftArm = null;
     Servo rightArm = null;
     Servo leftHand = null;
     Servo rightHand = null;

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
//        leftArm = hardwareMap.servo.get("left arm");
//        rightArm = hardwareMap.servo.get("right arm");
        leftHand = hardwareMap.servo.get("left hand");
        rightHand = hardwareMap.servo.get("right hand");
        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        spinArm.setDirection(DcMotor.Direction.FORWARD);

        // servo stuff
        //leftArm.scaleRange(0.2, 1.0);
        //rightArm.scaleRange(0.05, 0.8);

        //leftHand.scaleRange(0.0, 0.6);
        //rightHand.scaleRange(0.3, 1.0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

//        leftArm.setPosition(0.35);
//        rightArm.setPosition(0.65);
//
        leftHand.setPosition(1.0);
        rightHand.setPosition(0.0);

        int leftHandDirection = -1;
        int rightHandDirection = 1;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//            double leftArmPos = leftArm.getPosition();
//            double rightArmPos = rightArm.getPosition();
            double leftHandPos = leftHand.getPosition();
            double rightHandPos = rightHand.getPosition();

            // getdirection does not display unless you set the direction
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            if (leftHandPos != Double.NaN) {
                telemetry.addData("Status", "Left Hand: " + leftHandPos);
            }
            if (rightHandPos != Double.NaN) {
                telemetry.addData("Status", "Right Hand: " + rightHandPos);
            }
            telemetry.update();

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
             leftDrive.setPower(gamepad1.left_stick_y*DRIVE_MULTIPLE);
             rightDrive.setPower(gamepad1.right_stick_y*DRIVE_MULTIPLE);

            // Spin Arm setup
            if(gamepad2.right_trigger> DEADZONE){
                 //forward direction, launch mode
                 spinArm.setDirection(DcMotor.Direction.FORWARD);
                 if(gamepad2.right_bumper==true){
                     spinArm.setPower(gamepad2.right_trigger*SPIN_ARM_MINSPEED);
                 }
                 else {
                     spinArm.setPower(SPIN_ARM_MAXSPEED);
                     Thread.sleep(SPIN_ARM_ROTATE_TIME);
                     spinArm.setPower(SPIN_ARM_STOP);
                 }
            }
            else if(gamepad2.left_trigger> DEADZONE){
                //reverse direction, launch mode
                spinArm.setDirection(DcMotor.Direction.REVERSE);
                if(gamepad2.left_bumper==true){
                    spinArm.setPower(gamepad2.left_trigger*SPIN_ARM_MINSPEED);
                }
                else {
                    spinArm.setPower(SPIN_ARM_MAXSPEED);
                    Thread.sleep(SPIN_ARM_ROTATE_TIME);
                    spinArm.setPower(SPIN_ARM_STOP);
                }
            }
            else {
                spinArm.setPower(SPIN_ARM_STOP);
            }

            // Hand control code
            if(gamepad2.right_stick_y > DEADZONE){
                //gamepad 2 right joystick forward
                rightHand.setPosition(rightHand.getPosition() - HAND_SPEED);
            }
            else if(gamepad2.right_stick_y < (-1 * DEADZONE)){
                //gamepad 2 right joystick reverse
                rightHand.setPosition(rightHand.getPosition() + HAND_SPEED);
            }

            if(gamepad2.left_stick_y > DEADZONE){
                //gamepad 2 left joystick forward
                leftHand.setPosition(leftHand.getPosition() + HAND_SPEED);
            }
            else if(gamepad2.left_stick_y < (-1 * DEADZONE)){
                //gamepad 2 left joystick reverse
                leftHand.setPosition(leftHand.getPosition() - HAND_SPEED);
            }


//            if(gamepad1.right_trigger > DEADZONE){
//                rightHand.setPosition(rightHand.getPosition() + (HAND_SPEED * rightHandDirection));
//            }
//            else if ((gamepad1.right_trigger != 0) && (gamepad1.right_trigger < DEADZONE)) {
//                rightHandDirection *= -1;
//            }
//            else if(gamepad1.left_trigger > DEADZONE){
//                leftHand.setPosition((leftHand.getPosition() + HAND_SPEED) * leftHandDirection);
//            }
//            else if ((gamepad1.left_trigger != 0) && (gamepad1.left_trigger < DEADZONE)) {
//                leftHandDirection *= -1;
//            }

//            // Left/Right arm for cap ball lift
//            if(gamepad1.right_trigger>0.25){
//                //leftArm.setDirection(Servo.Direction.FORWARD);
//                leftArm.setPosition(leftArm.getPosition() - .1d);
//                //rightArm.setDirection(Servo.Direction.REVERSE);
//                rightArm.setPosition(rightArm.getPosition() + .1d);
//            }
//            else if(gamepad1.left_trigger>0.25){
//                //leftArm.setDirection(Servo.Direction.REVERSE);
//                leftArm.setPosition(leftArm.getPosition() + .1d);
//                //rightArm.setDirection(Servo.Direction.FORWARD);
//                rightArm.setPosition(rightArm.getPosition() - .1d);
//            }

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
