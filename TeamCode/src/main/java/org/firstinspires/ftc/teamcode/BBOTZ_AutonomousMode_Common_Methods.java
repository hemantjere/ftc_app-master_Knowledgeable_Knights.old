package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BBOTZ_AutonomousMode_Common_Methods {

    private double MAX_DRIVE_SPEED = .2d;
    private double STOP_DRIVE = 0d;
    private double SPIN_ARM_MAXSPEED = 1.0d;
    private double SPIN_ARM_MINSPEED = .3d;
    private double SPIN_ARM_STOP = 0d;
    private long SPIN_ARM_SLOW_ROTATE_TIME = 500;
    private long SPIN_ARM_FAST_ROTATE_TIME = 800;
    private double ZIPTIE_MOTOR_SPEED = 1d;
    private double ZIPTIE_MOTOR_STOP = 0d;

    private long STOP_TIME = 500;

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftDrive = null;
    DcMotor rightDrive = null;
    DcMotor spinArm = null;
    DcMotor ziptieMotor = null;

    DcMotorController spinArmController;
    int spinArmPort;

    BBOTZ_AutonomousMode_Common_Methods(HardwareMap hardwareMap) {
        this.leftDrive = leftDrive;

        this.rightDrive = rightDrive;
        this.spinArm = spinArm;

         /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftDrive = hardwareMap.dcMotor.get("left drive wheel");
        rightDrive = hardwareMap.dcMotor.get("right drive wheel");
        spinArm = hardwareMap.dcMotor.get("spin arm");
        ziptieMotor = hardwareMap.dcMotor.get("ziptie motor");
        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);  // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors
        spinArm.setDirection(DcMotor.Direction.REVERSE);
        ziptieMotor.setDirection(DcMotor.Direction.REVERSE);

        //Remove once added to auto mode
        spinArmController = spinArm.getController();
        spinArmPort = spinArm.getPortNumber();
        spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.RUN_USING_ENCODER);
        resetEncoder();
    }

    protected void driveForward(long driveTime) throws InterruptedException {
        leftDrive.setPower(MAX_DRIVE_SPEED);
        rightDrive.setPower(MAX_DRIVE_SPEED);
        Thread.sleep(driveTime);
        leftDrive.setPower(STOP_DRIVE);
        rightDrive.setPower(STOP_DRIVE);
        Thread.sleep(STOP_TIME);
    }

    protected void turnLeft(long driveTime) throws InterruptedException{
        rightDrive.setPower(MAX_DRIVE_SPEED);
        Thread.sleep(driveTime);
        rightDrive.setPower(STOP_DRIVE);
        Thread.sleep(STOP_TIME);
    }

    protected void tankTurnLeft(long driveTime) throws InterruptedException{
        rightDrive.setPower(MAX_DRIVE_SPEED);
        leftDrive.setPower(-MAX_DRIVE_SPEED);
        Thread.sleep(driveTime);
        rightDrive.setPower(STOP_DRIVE);
        leftDrive.setPower(STOP_DRIVE);
        Thread.sleep(STOP_TIME);
    }

    protected void turnRight(long driveTime) throws InterruptedException {
        leftDrive.setPower(MAX_DRIVE_SPEED);
        Thread.sleep(driveTime);
        leftDrive.setPower(STOP_DRIVE);
        Thread.sleep(STOP_TIME);
    }

    protected void tankTurnRight(long driveTime) throws InterruptedException{
        rightDrive.setPower(-MAX_DRIVE_SPEED);
        leftDrive.setPower(MAX_DRIVE_SPEED);
        Thread.sleep(driveTime);
        rightDrive.setPower(STOP_DRIVE);
        leftDrive.setPower(STOP_DRIVE);
        Thread.sleep(STOP_TIME);
    }

    protected void launchBall() throws InterruptedException {
        startSpinArmLaunch();
        ziptieRun();
        startSpinArmHome();
        startSpinArmLaunch();
        startSpinArmHome();
        ziptieStop();
    }

    protected void startSpinArmLaunch() throws InterruptedException {
        spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.RUN_USING_ENCODER);

        int prevPos = Integer.MAX_VALUE;

        spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.RUN_USING_ENCODER);
        spinArm.setTargetPosition(320);
        spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.RUN_TO_POSITION);
        spinArm.setPower(SPIN_ARM_MAXSPEED);
        long startTime = System.currentTimeMillis();
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

            if (spinArm.getCurrentPosition() >= 320) {
                break;
            }
        }

        spinArm.setPower(SPIN_ARM_STOP);

        Thread.sleep(1000);
    }

    protected void startSpinArmHome () throws InterruptedException {
        spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.RUN_USING_ENCODER);

        int prevPos = Integer.MAX_VALUE;

        spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.RUN_USING_ENCODER);
        spinArm.setTargetPosition(700);
        spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.RUN_TO_POSITION);
        spinArm.setPower(SPIN_ARM_MINSPEED);
        long startTime = System.currentTimeMillis();
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

            if (spinArm.getCurrentPosition() >= 700) {
                break;
            }
        }

        spinArm.setPower(SPIN_ARM_STOP);
        resetEncoder();

        Thread.sleep(1000);
    }

    protected void driveBackward(long driveTime) throws InterruptedException {
        leftDrive.setPower(-MAX_DRIVE_SPEED);
        rightDrive.setPower(-MAX_DRIVE_SPEED);
        Thread.sleep(driveTime);
        leftDrive.setPower(STOP_DRIVE);
        rightDrive.setPower(STOP_DRIVE);
        Thread.sleep(STOP_TIME);
    }

    protected void ziptieRun (){
        ziptieMotor.setPower(ZIPTIE_MOTOR_SPEED);
    }

    protected void ziptieStop (){
        ziptieMotor.setPower(ZIPTIE_MOTOR_STOP);
    }

    protected void resetEncoder (){ spinArmController.setMotorMode(spinArmPort, DcMotor.RunMode.STOP_AND_RESET_ENCODER); }

}
