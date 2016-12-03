package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BBOTZ_AutonomousMode_Base {

    private double MAX_DRIVE_SPEED = .2d;
    private double STOP_DRIVE = 0d;
    private double SPIN_ARM_FORWARD_MAXSPEED = 0.8d;
    private double SPIN_ARM_FORWARD_SLOWSPEED = .3d;
    private double SPIN_ARM_STOP = 0d;
    private long SPIN_ARM_SLOW_ROTATE_TIME = 500;
    private long SPIN_ARM_FAST_ROTATE_TIME = 800;
    private long ZIPTIE_MOTOR_SPEED = 1;
    private long ZIPTIE_MOTOR_STOP = 0;

    private long STOP_TIME = 500;

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftDrive = null;
    DcMotor rightDrive = null;
    DcMotor spinArm = null;
    DcMotor ziptieMotor = null;
    BBOTZ_AutonomousMode_Base(HardwareMap hardwareMap) {
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
        ziptieMotor.setPower(ZIPTIE_MOTOR_SPEED);
        startSpinArm(SPIN_ARM_FORWARD_MAXSPEED, SPIN_ARM_FAST_ROTATE_TIME);
        stopSpinArm();
        ziptieMotor.setPower(ZIPTIE_MOTOR_STOP);
        //startSpinArm(SPIN_ARM_FORWARD_SLOWSPEED, SPIN_ARM_SLOW_ROTATE_TIME);
        //stopSpinArm();
        //startSpinArm(SPIN_ARM_FORWARD_MAXSPEED, SPIN_ARM_FAST_ROTATE_TIME);
        //stopSpinArm();
    }

    private void startSpinArm(double spinSpeed, long spinTime) throws InterruptedException {
        spinArm.setPower(spinSpeed);
        Thread.sleep(spinTime);
    }

    private void stopSpinArm() throws InterruptedException {
        spinArm.setPower(SPIN_ARM_STOP);
        Thread.sleep(STOP_TIME);
    }

    protected void driveBackward(long driveTime) throws InterruptedException {
        leftDrive.setPower(-MAX_DRIVE_SPEED);
        rightDrive.setPower(-MAX_DRIVE_SPEED);
        Thread.sleep(driveTime);
        leftDrive.setPower(STOP_DRIVE);
        rightDrive.setPower(STOP_DRIVE);
        Thread.sleep(STOP_TIME);
    }
}
