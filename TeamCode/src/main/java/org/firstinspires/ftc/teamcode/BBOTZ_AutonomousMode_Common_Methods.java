package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BBOTZ_AutonomousMode_Common_Methods {

    private double MAX_DRIVE_SPEED = .1d;
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
    private int ODS_TEST_TIME = 10;
    private double ODS_VALUE = 0.05d;

    private long STOP_TIME = 500;

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftDrive = null;
    DcMotor rightDrive = null;
    DcMotor spinArm = null;
    DcMotor ziptieMotor = null;
    OpticalDistanceSensor odsSensor = null;
    DcMotorController spinArmController;
    int spinArmPort;

    BBOTZ_AutonomousMode_Common_Methods(HardwareMap hardwareMap) {
        //Initialize hardware
        leftDrive = hardwareMap.dcMotor.get("left drive wheel");
        rightDrive = hardwareMap.dcMotor.get("right drive wheel");
        spinArm = hardwareMap.dcMotor.get("spin arm");
        ziptieMotor = hardwareMap.dcMotor.get("ziptie motor");
        odsSensor = hardwareMap.opticalDistanceSensor.get("ods");

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
    }

    //drive forwards
    protected void driveForwardUsingTime(long driveTime) throws InterruptedException {
        leftDrive.setPower(MAX_DRIVE_SPEED);
        rightDrive.setPower(MAX_DRIVE_SPEED);
        Thread.sleep(driveTime);
        leftDrive.setPower(STOP_DRIVE);
        rightDrive.setPower(STOP_DRIVE);
        Thread.sleep(STOP_TIME);
    }

    protected void driveForwardUsingODS() throws InterruptedException {
        leftDrive.setPower(MAX_DRIVE_SPEED);
        rightDrive.setPower(MAX_DRIVE_SPEED);
        while(odsSensor.getLightDetected() < ODS_VALUE) {
            Thread.sleep(ODS_TEST_TIME);
        }
        leftDrive.setPower(STOP_DRIVE);
        rightDrive.setPower(STOP_DRIVE);
        Thread.sleep(STOP_TIME);
    }

    //turn left
    protected void turnLeft(long driveTime) throws InterruptedException{
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

    //drive backwards
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
