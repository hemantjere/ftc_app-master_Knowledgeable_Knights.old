package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BBOTZ_AutonomousMode_Base {

    private double MAX_DRIVE_SPEED = .2d;
    private double STOP_DRIVE = 0d;
    private double SPIN_ARM_FORWARD_MAXSPEED = 1d;
    private double SPIN_ARM_STOP = 0d;
    private long SPIN_ARM_ROTATE_TIME = 800;

    private long STOP_TIME = 500;

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftDrive = null;
    DcMotor rightDrive = null;
    DcMotor spinArm = null;

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

    protected void turnRight(long driveTime) throws InterruptedException {
        leftDrive.setPower(MAX_DRIVE_SPEED);
        Thread.sleep(driveTime);
        leftDrive.setPower(STOP_DRIVE);
        Thread.sleep(STOP_TIME);
    }

    protected void launchBall() throws InterruptedException {
        spinArm.setPower(SPIN_ARM_FORWARD_MAXSPEED);
        Thread.sleep(SPIN_ARM_ROTATE_TIME);
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
