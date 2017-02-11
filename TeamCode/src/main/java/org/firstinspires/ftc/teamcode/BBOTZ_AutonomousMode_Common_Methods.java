package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BBOTZ_AutonomousMode_Common_Methods {

    private double MAX_DRIVE_SPEED = .1d;
    private double MAX_DRIVE_SPEED_BEACON = .4d;
    private double MAX_DRIVE_SPEED_PRESS_BEACON = 1d;
    private double STOP_DRIVE = 0d;
    private double SPIN_ARM_FIRST_SHOT = 1.0d;
    private double SPIN_ARM_SECOND_SHOT = 1.0d;
    private double SPIN_ARM_MINSPEED = .3d;
    private double SPIN_ARM_STOP = 0d;
    private int SPIN_ARM_CHECK_TIME = 50;
    private long SPIN_ARM_SLOW_ROTATE_TIME = 500;
    private long SPIN_ARM_FAST_ROTATE_TIME = 800;
    private double ZIPTIE_MOTOR_SPEED = 1d;
    private double ZIPTIE_MOTOR_STOP = 0d;
    private int ODS_TEST_TIME = 50;
    private double ODS_TAPE_VALUE = 0.05d;
    private double TIME_TO_MOVE_TOWARDS_BEACON = 20000;
    private double ADJUSTMENT_ERROR = .1;
    private int MAX_BEACON_PRESSES = 5;
    private double PREFERRED_LIGHT_VALUE = .5d;

    protected static int GET_RED = 0;
    protected static int GET_BLUE = 1;

    private long STOP_TIME = 500;

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftDrive = null;
    DcMotor rightDrive = null;
    DcMotor spinArm = null;
    DcMotor ziptieMotor = null;
    OpticalDistanceSensor odsSensor = null;
    ColorSensor colorSensor = null;
    DcMotorController spinArmController;
    int spinArmPort;

    BBOTZ_AutonomousMode_Common_Methods(HardwareMap hardwareMap) {
        //Initialize hardware
        leftDrive = hardwareMap.dcMotor.get("left drive wheel");
        rightDrive = hardwareMap.dcMotor.get("right drive wheel");
        spinArm = hardwareMap.dcMotor.get("spin arm");
        ziptieMotor = hardwareMap.dcMotor.get("ziptie motor");
        odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
        colorSensor = hardwareMap.colorSensor.get("color sensor");

        //Set directions for motors
        leftDrive.setDirection(DcMotor.Direction.REVERSE);  // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors
        spinArm.setDirection(DcMotor.Direction.REVERSE);
        ziptieMotor.setDirection(DcMotor.Direction.REVERSE);

        //Spin arm setup
        spinArmController = spinArm.getController();
        spinArmPort = spinArm.getPortNumber();
        spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.RUN_USING_ENCODER);
        resetEncoder();

        //Turn LED off
        turnColorSensorLedOff();
    }

    // drive beacon
    protected void driveForwardUsingTimeBeacon(long driveTime) throws InterruptedException {
        leftDrive.setPower(MAX_DRIVE_SPEED_BEACON);
        rightDrive.setPower(MAX_DRIVE_SPEED_BEACON);
        Thread.sleep(driveTime);
        leftDrive.setPower(STOP_DRIVE);
        rightDrive.setPower(STOP_DRIVE);
        Thread.sleep(STOP_TIME);
    }

    // drive forwards
    protected void driveForwardUsingTime(long driveTime) throws InterruptedException {
        leftDrive.setPower(MAX_DRIVE_SPEED);
        rightDrive.setPower(MAX_DRIVE_SPEED);
        Thread.sleep(driveTime);
        leftDrive.setPower(STOP_DRIVE);
        rightDrive.setPower(STOP_DRIVE);
        Thread.sleep(STOP_TIME);
    }

    //drive backwards
    protected void driveBackwardUsingTime(long driveTime) throws InterruptedException {
        leftDrive.setPower(-MAX_DRIVE_SPEED);
        rightDrive.setPower(-MAX_DRIVE_SPEED);
        Thread.sleep(driveTime);
        leftDrive.setPower(STOP_DRIVE);
        rightDrive.setPower(STOP_DRIVE);
        Thread.sleep(STOP_TIME);
    }

    protected void driveBackwardUsingTimePressBeacon(long driveTime) throws InterruptedException{
        leftDrive.setPower(-MAX_DRIVE_SPEED_PRESS_BEACON);
        rightDrive.setPower(-MAX_DRIVE_SPEED_PRESS_BEACON);
        Thread.sleep(driveTime);
        leftDrive.setPower(STOP_DRIVE);
        rightDrive.setPower(STOP_DRIVE);
        Thread.sleep(STOP_TIME);
    }

    protected void driveForwardUsingODS() throws InterruptedException {
        leftDrive.setPower(MAX_DRIVE_SPEED);
        rightDrive.setPower(MAX_DRIVE_SPEED);

        while(odsSensor.getLightDetected() < ODS_TAPE_VALUE) {
            Thread.sleep(ODS_TEST_TIME);
        }

        leftDrive.setPower(STOP_DRIVE);
        rightDrive.setPower(STOP_DRIVE);
        Thread.sleep(STOP_TIME);
    }

    protected void followTapeUsingODS(Telemetry telemetry) throws InterruptedException {
        double currentODSReading;
        double odsDelta;
        double powerToLeftWheel = -.075D;
        double powerToRightWheel = -.075D;
        boolean lineFollowerActivated = false;

        double startTime = runtime.startTime();

        while (true) {
            currentODSReading = odsSensor.getLightDetected();
            odsDelta = PREFERRED_LIGHT_VALUE - currentODSReading;

            if (odsDelta < 0)
                lineFollowerActivated = true;

            if (lineFollowerActivated) {
                    // odsDelta will be either positive or negative
                    powerToLeftWheel = -.075d + (odsDelta * .075d);
                    powerToRightWheel = -.075d - (odsDelta * .075d);
            }

            telemetry.addData("currentODSReading", currentODSReading);
            telemetry.addData("odsDelta", odsDelta);
            telemetry.addData("powerToLeftWheel", powerToLeftWheel);
            telemetry.addData("powerToRightWheel", powerToRightWheel);
            telemetry.update();

            leftDrive.setPower(powerToLeftWheel);
            rightDrive.setPower(powerToRightWheel);
            Thread.sleep(ODS_TEST_TIME);

            // Stop moving towards beacon after x seconds
            double currentTime = runtime.time();
            if (currentTime > (startTime + TIME_TO_MOVE_TOWARDS_BEACON)) {
                break;
            }
        }

        leftDrive.setPower(STOP_DRIVE);
        rightDrive.setPower(STOP_DRIVE);
        Thread.sleep(STOP_TIME);
    }

    protected void pressBeaconForColor(int desiredColor) throws InterruptedException {
        boolean done = false;
        int numberOfTries = 0;

        while (! done) {
            // Press the beacon button
            driveBackwardUsingTimePressBeacon(1000);

            // Test for desired color
            if (desiredColor == 1) {
                if (colorSensor.red() > colorSensor.blue()) {
                    done = true;
                }
            }
            else if (desiredColor == 1) {
                if (colorSensor.blue() > colorSensor.red()) {
                    done = true;
                }
            }

            // Drive forward after color checked to press again
            driveForwardUsingTime(800);

            // If number of tries exceeded, stop
            numberOfTries++;
            if (numberOfTries > MAX_BEACON_PRESSES)
                done = true;
        }
    }

    //turn left
    protected void turnLeft(long driveTime) throws InterruptedException {
        rightDrive.setPower(MAX_DRIVE_SPEED);
        Thread.sleep(driveTime);
        rightDrive.setPower(STOP_DRIVE);
        Thread.sleep(STOP_TIME);
    }

    //tank turn left
    protected void tankTurnLeft(long driveTime) throws InterruptedException{
        rightDrive.setPower(MAX_DRIVE_SPEED);
        leftDrive.setPower(-MAX_DRIVE_SPEED);
        Thread.sleep(driveTime);
        rightDrive.setPower(STOP_DRIVE);
        leftDrive.setPower(STOP_DRIVE);
        Thread.sleep(STOP_TIME);
    }

    //turn right
    protected void turnRight(long driveTime) throws InterruptedException {
        leftDrive.setPower(MAX_DRIVE_SPEED);
        Thread.sleep(driveTime);
        leftDrive.setPower(STOP_DRIVE);
        Thread.sleep(STOP_TIME);
    }

    //tank turn right
    protected void tankTurnRight(long driveTime) throws InterruptedException{
        rightDrive.setPower(-MAX_DRIVE_SPEED);
        leftDrive.setPower(MAX_DRIVE_SPEED);
        Thread.sleep(driveTime);
        rightDrive.setPower(STOP_DRIVE);
        leftDrive.setPower(STOP_DRIVE);
        Thread.sleep(STOP_TIME);
    }

    //launch two balls
    protected void launchBall() throws InterruptedException {
        startSpinArmLaunch(SPIN_ARM_FIRST_SHOT, 280);
        ziptieRun();
        startSpinArmHome();
        startSpinArmLaunch(SPIN_ARM_SECOND_SHOT, 280);
        startSpinArmHome();
        ziptieStop();
    }

    //launch position spin arm
    protected void startSpinArmLaunch(double spinSpeed, int spinLocation) throws InterruptedException {
        //automatic throw mode

        spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.RUN_USING_ENCODER);

        int prevPos = Integer.MAX_VALUE;

        //320 is launch position
        spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.RUN_USING_ENCODER);
        spinArm.setTargetPosition(spinLocation);
        spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.RUN_TO_POSITION);
        spinArm.setPower(spinSpeed);
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

            if (spinArm.getCurrentPosition() >= spinLocation) {
                break;
            }
        }

        //Stop Spin Arm
        spinArm.setPower(SPIN_ARM_STOP);

        //One second delay
        Thread.sleep(1000);
    }

    //home position spin arm
    protected void startSpinArmHome () throws InterruptedException {
        spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.RUN_USING_ENCODER);

        int prevPos = Integer.MAX_VALUE;

        //700 is home position
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
        }

        //Stop Spin Arm
        spinArm.setPower(SPIN_ARM_STOP);
        resetEncoder();

        //One second delay
        Thread.sleep(1000);
    }

    protected void turnColorSensorLedOff() { colorSensor.enableLed(false);}

    protected void ziptieRun (){
        ziptieMotor.setPower(ZIPTIE_MOTOR_SPEED);
    }

    protected void ziptieStop (){
        ziptieMotor.setPower(ZIPTIE_MOTOR_STOP);
    }

    protected void resetEncoder (){ spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.STOP_AND_RESET_ENCODER); }

}
