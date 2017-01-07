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
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Template: Tank Drive Control OpMode", group="OpMode Control")  // @Autonomous(...) is the other common choice

// @Disabled
public class BBOTZ_OpMode_Tank_Mode_Control extends LinearOpMode {

    private double SPIN_ARM_MAXSPEED = .8d;
    private double SPIN_ARM_MINSPEED = .2d;
    private double SPIN_ARM_STOP = 0d;
    private double ZIPTIE_MOTOR_SPEED = 1d;
    private double ZIPTIE_MOTOR_STOP = 0d;
    private double DEADZONE = .1d;
    private long SPIN_ARM_ROTATE_TIME = 200;
    private double ACCELERATION_RATE = .25;
    private long BEACON_DPAD_RUN_TIME = 500;
    private long Y_TOGGLE_RUN_TIME = 800;

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

        //Initialize hardware
        leftDrive = hardwareMap.dcMotor.get("left drive wheel");
        rightDrive = hardwareMap.dcMotor.get("right drive wheel");
        spinArm = hardwareMap.dcMotor.get("spin arm");
        leftHand = hardwareMap.servo.get("left hand");
        rightHand = hardwareMap.servo.get("right hand");
        ziptieMotor = hardwareMap.dcMotor.get("ziptie motor");

        //Set directions for motors
        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        spinArm.setDirection(DcMotor.Direction.REVERSE);
        ziptieMotor.setDirection(DcMotor.Direction.REVERSE);

        //Variable for y toggle
        Boolean toggleYButtonZiptie = false;

        //Setup encoder variables
        DcMotorController spinArmController = spinArm.getController();
        int spinArmPort = spinArm.getPortNumber();
        spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.RUN_USING_ENCODER);

        //Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Telemetry display on driver station
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Status", "gamepad1: " + gamepad1.toString());
            telemetry.addData("Status", "gamepad2: " + gamepad2.toString());
            telemetry.addData("Status", "Arm Position:" + spinArm.getCurrentPosition());
            telemetry.update();

            //Driving controls
            if (gamepad1.left_bumper == true) {
                //Slow speed
                leftDrive.setPower(gamepad1.left_stick_y*DRIVE_MULTIPLE);
                rightDrive.setPower(gamepad1.right_stick_y*DRIVE_MULTIPLE);
            }
            else {
                //Normal speed
                leftDrive.setPower(gamepad1.left_stick_y);
                rightDrive.setPower(gamepad1.right_stick_y);
            }

            //Beacon press forward & backward movement
            if (gamepad1.dpad_up == true) {
                //Forward movement for BEACON_DPAD_RUN_TIME
                leftDrive.setPower(-1);
                rightDrive.setPower(-1);
                sleep(BEACON_DPAD_RUN_TIME);
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }
            if (gamepad1.dpad_down == true) {
                //Backward movement for BEACON_DPAD_RUN_TIME
                leftDrive.setPower(1);
                rightDrive.setPower(1);
                sleep(BEACON_DPAD_RUN_TIME);
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }

            //Encoder movement
            if(gamepad2.right_bumper == true){
                //automatic throw mode

                int prevPos = Integer.MAX_VALUE;

                //Run ziptie motor during arm movement
                ziptieRun();

                //330 is launch position
                spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.RUN_USING_ENCODER);
                spinArm.setTargetPosition(330);
                spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.RUN_TO_POSITION);
                spinArm.setPower(SPIN_ARM_MAXSPEED);
                long startTime = System.currentTimeMillis();

                //while() loop for motor burnout prevention
                while (spinArm.isBusy()) {
                    // wait for arm to throw
                    long currentTime = System.currentTimeMillis();
                    if (currentTime > startTime + 50) {
                        if (prevPos == spinArm.getCurrentPosition()) {
                            break;
                        }

                        startTime = currentTime;
                        prevPos = spinArm.getCurrentPosition();
                    }

                    if (spinArm.getCurrentPosition() >= 330) {
                        break;
                    }

                    telemetry.update();

                }

                //Stop ziptie motor during arm movement
                ziptieStop();

                //Stop Spin Arm
                spinArm.setPower(SPIN_ARM_STOP);

                //One second delay
                sleep(1000);
            }
            else if(gamepad2.left_bumper == true){
                // automatic home reset mode
                int prevPos = Integer.MAX_VALUE;

                //Run ziptie motor during arm movement
                ziptieRun();

                //690 is home position
                spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.RUN_USING_ENCODER);
                spinArm.setTargetPosition(690);
                spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.RUN_TO_POSITION);
                spinArm.setPower(SPIN_ARM_MINSPEED);
                long startTime = System.currentTimeMillis();

                //while() loop for motor burnout prevention
                while (spinArm.isBusy()) {
                    // wait for arm to throw
                    long currentTime = System.currentTimeMillis();
                    if (currentTime > startTime + 100) {
                        if (prevPos == spinArm.getCurrentPosition()) {
                            break;
                        }

                        startTime = currentTime;
                        prevPos = spinArm.getCurrentPosition();
                    }

                    if (spinArm.getCurrentPosition() >= 690) {
                        break;
                    }

                    telemetry.update();
                }

                //Stop ziptie motor during arm movement
                ziptieStop();

                //Stop Spin Arm
                spinArm.setPower(SPIN_ARM_STOP);
                spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                //One second delay
                sleep(1000);
            }

            //Manual Spin Arm movement
            else if(gamepad2.right_trigger > DEADZONE){
                //manual forward mode
                spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                spinArm.setDirection(DcMotor.Direction.REVERSE);
                spinArm.setPower(gamepad2.right_trigger*SPIN_ARM_MINSPEED);
            }
            else if(gamepad2.left_trigger > DEADZONE){
                //manual backward mode
                spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                spinArm.setDirection(DcMotor.Direction.FORWARD);
                spinArm.setPower(gamepad2.left_trigger*SPIN_ARM_MINSPEED);
            }
            //Make sure that motor does not run while pressure is not applied
            else {
                spinArm.setPower(SPIN_ARM_STOP);
            }

            //Ziptie Motor setup
            if(gamepad2.y == true){
                toggleYButtonZiptie = !toggleYButtonZiptie;
                sleep(Y_TOGGLE_RUN_TIME);
            }

            if (toggleYButtonZiptie == true) {
                ziptieRun();
            }
            else {
                ziptieStop();
            }

            //Reset to zero button
            if(gamepad2.x == true){
                spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.RUN_USING_ENCODER);
            }

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    //Ziptie Motor Functions
    public void ziptieRun (){ziptieMotor.setPower(ZIPTIE_MOTOR_SPEED);}
    public void ziptieStop (){ziptieMotor.setPower(ZIPTIE_MOTOR_STOP);}
}