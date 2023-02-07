package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "LiamAuto", group = "Drew's Auto" )

public class LiamAuto extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontleftDrive = null;
    private DcMotor frontrightDrive = null;
    private DcMotor backleftDrive = null;
    private DcMotor backrightDrive = null;
    private DcMotor middleslideDrive = null;
    private Servo rightgripperDrive = null;
    private Servo leftgripperDrive = null;
    public IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        frontleftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontrightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backleftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backrightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        middleslideDrive = hardwareMap.get(DcMotor.class, "middle_slides_drive");

        // Initialize motors and set their directions
        frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontleftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontrightDrive.setDirection(DcMotor.Direction.FORWARD);
        backleftDrive.setDirection(DcMotor.Direction.REVERSE);
        backrightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        middleslideDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //imu
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        double movementSpeed = .2;

        imu.resetYaw();
        waitForStart();

        moveSimpleTargetPosition(.25,300,1,10);
        moveSimpleTargetPosition(.25,5000,1,500);
        setSlider(1,3, 500);
        moveSimpleTargetPosition(.25,2500,1,500);
        setSlider(1,0, 500);


    }
    public void moveSimpleTargetPosition(double speed, int distance, int direction, int sleep) {

        frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (direction == 1) {
            frontleftDrive.setTargetPosition(distance);
            frontrightDrive.setTargetPosition(distance);
            backleftDrive.setTargetPosition(distance);
            backrightDrive.setTargetPosition(distance);

            frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontleftDrive.setPower(speed);
            frontrightDrive.setPower(speed);
            backleftDrive.setPower(speed);
            backrightDrive.setPower(speed);
        }
        if (direction == 2) {
            frontleftDrive.setTargetPosition(-distance);
            frontrightDrive.setTargetPosition(distance);
            backleftDrive.setTargetPosition(distance);
            backrightDrive.setTargetPosition(-distance);

            frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontleftDrive.setPower(-speed);
            frontrightDrive.setPower(speed);
            backleftDrive.setPower(speed);
            backrightDrive.setPower(-speed);
        }
        if (direction == 3) {
            frontleftDrive.setTargetPosition(-distance);
            frontrightDrive.setTargetPosition(-distance);
            backleftDrive.setTargetPosition(-distance);
            backrightDrive.setTargetPosition(-distance);

            frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontleftDrive.setPower(-speed);
            frontrightDrive.setPower(-speed);
            backleftDrive.setPower(-speed);
            backrightDrive.setPower(-speed);
        }
        if (direction == 4) {
            frontleftDrive.setTargetPosition(distance);
            frontrightDrive.setTargetPosition(-distance);
            backleftDrive.setTargetPosition(-distance);
            backrightDrive.setTargetPosition(distance);

            frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontleftDrive.setPower(speed);
            frontrightDrive.setPower(-speed);
            backleftDrive.setPower(-speed);
            backrightDrive.setPower(speed);
        }


        while (frontleftDrive.isBusy() && frontrightDrive.isBusy() && backleftDrive.isBusy() && backrightDrive.isBusy()) {
            telemetry();
            // Wait for the motors to reach their target positions and updates telemetry
        }

        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);

        sleep(sleep);
    }
    public void rotationCheck (double speed, double desiredRotationAngle) {
        if ((imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < desiredRotationAngle) {
            frontleftDrive.setPower(speed);
            frontrightDrive.setPower(speed);
            backleftDrive.setPower(speed);
            backrightDrive.setPower(speed);
            telemetry();
        }
    }
    public void rotate(double speed, int distance, int direction) {
        if (direction == 0) {
            frontleftDrive.setTargetPosition(-distance);
            frontrightDrive.setTargetPosition(distance);
            backleftDrive.setTargetPosition(-distance);
            backrightDrive.setTargetPosition(distance);

            frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontleftDrive.setPower(-speed);
            frontrightDrive.setPower(speed);
            backleftDrive.setPower(-speed);
            backrightDrive.setPower(speed);
        }
        if (direction == 1) {
            frontleftDrive.setTargetPosition(distance);
            frontrightDrive.setTargetPosition(-distance);
            backleftDrive.setTargetPosition(distance);
            backrightDrive.setTargetPosition(-distance);

            frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontleftDrive.setPower(speed);
            frontrightDrive.setPower(-speed);
            backleftDrive.setPower(speed);
            backrightDrive.setPower(-speed);
        }
        while (frontleftDrive.isBusy() && frontrightDrive.isBusy() && backleftDrive.isBusy() && backrightDrive.isBusy()) {
            telemetry();
            // Wait for the motors to reach their target positions and updates telemetry
        }

        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);

        sleep(1000);

    }
    public void setSlider(double speed, double level, int sleep) {
        //middleslideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //middleslideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double position = 0;
        if (level == 1) {position = 70;}
        if (level == 2) {position = 2000;}
        if (level == 3) {position = 4000;}
        if (middleslideDrive.getCurrentPosition() < position) {
            while (middleslideDrive.getCurrentPosition()< position) {middleslideDrive.setPower(-speed);}
            telemetry();

        }
        if (middleslideDrive.getCurrentPosition() > position) {
            while (middleslideDrive.getCurrentPosition()> position) {middleslideDrive.setPower(speed);}
            telemetry();
        }
        middleslideDrive.setPower(0);
        sleep(sleep);
    }
    public void telemetry() {
        telemetry.addData("Run Time", runtime.toString());
        telemetry.addData("Front Right Encoder", frontrightDrive.getCurrentPosition());
        telemetry.addData("Front Left Encoder", frontleftDrive.getCurrentPosition());
        telemetry.addData("Back Right Encoder", backrightDrive.getCurrentPosition());
        telemetry.addData("Back Left Encoder", backleftDrive.getCurrentPosition());
        telemetry.addData("IMU Z Angle", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.update();
    }
}


