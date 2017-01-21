//TANK MODE CONTROL
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

    private double SPIN_ARM_MAXSPEED = 1.0d;
    private double SPIN_ARM_MINSPEED = .2d;
    private double SPIN_ARM_STOP = 0d;
    private int SPIN_ARM_CHECK_TIME = 50;
    private double ZIPTIE_MOTOR_SPEED = 1d;
    private double ZIPTIE_MOTOR_STOP = 0d;
    private double DEADZONE = .1d;
    private long BEACON_DPAD_RUN_TIME = 500;
    private long Y_TOGGLE_RUN_TIME = 800;

    private float DRIVE_MULTIPLE = .2f;

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

                //330 is launch position
                spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.RUN_USING_ENCODER);
                spinArm.setTargetPosition(280);
                spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.RUN_TO_POSITION);
                spinArm.setPower(SPIN_ARM_MAXSPEED);
                long startTime = System.currentTimeMillis();

                //while() loop for motor burnout prevention
                while (spinArm.isBusy()) {
                    // wait for arm to throw
                    long currentTime = System.currentTimeMillis();
                    if (currentTime > startTime + SPIN_ARM_CHECK_TIME) {
                        if (prevPos == spinArm.getCurrentPosition()) {
                            break;
                        }

                        startTime = currentTime;
                        prevPos = spinArm.getCurrentPosition();
                    }

                    if (spinArm.getCurrentPosition() >= 280) {
                        break;
                    }

                    telemetry.update();

                }

                //Stop Spin Arm
                spinArm.setPower(SPIN_ARM_STOP);

                //One second delay
                sleep(1000);
            }
            else if(gamepad2.left_bumper == true){
                // automatic home reset mode
                int prevPos = Integer.MAX_VALUE;

                //690 is home position
                spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.RUN_USING_ENCODER);
                spinArm.setTargetPosition(700);
                spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.RUN_TO_POSITION);
                spinArm.setPower(SPIN_ARM_MINSPEED);
                long startTime = System.currentTimeMillis();

                //while() loop for motor burnout prevention
                while (spinArm.isBusy()) {
                    // wait for arm to throw
                    long currentTime = System.currentTimeMillis();
                    if (currentTime > startTime + SPIN_ARM_CHECK_TIME) {
                        if (prevPos == spinArm.getCurrentPosition()) {
                            break;
                        }

                        startTime = currentTime;
                        prevPos = spinArm.getCurrentPosition();
                    }

                    if (spinArm.getCurrentPosition() >= 700) {
                        break;
                    }

                    telemetry.update();
                }

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