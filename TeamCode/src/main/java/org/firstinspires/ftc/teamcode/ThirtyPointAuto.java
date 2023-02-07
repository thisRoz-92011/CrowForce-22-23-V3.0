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
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "ThirtyPointAuto", group = "OpenCV Autos" )

public class ThirtyPointAuto extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    public DcMotor middleSlideDrive = null;
    private boolean hasRun = false;
    public DcMotor frontleftDrive = null;
    public DcMotor frontrightDrive = null;
    public DcMotor backleftDrive = null;
    public DcMotor backrightDrive = null;
    public DcMotor middleslideDrive = null;
    public Servo rightgripperDrive = null;
    public Servo leftgripperDrive = null;


    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagSize = 0.166;

    int tagPosition = 0;

    int tagOfInterest1 = 1; // Tag ID 1 from the 36h11 family
    int tagOfInterest2 = 2; // Tag ID 2 from the 36h11 family
    int tagOfInterest3 = 3; // Tag ID 3 from the 36h11 family
    public IMU imu;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        // Motors and Servos
        frontleftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontrightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backleftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backrightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        middleSlideDrive = hardwareMap.get(DcMotor.class, "middle_slides_drive");

        rightgripperDrive = hardwareMap.get(Servo.class, "right_gripper_drive");
        leftgripperDrive = hardwareMap.get(Servo.class, "left_gripper_drive");

        frontleftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontrightDrive.setDirection(DcMotor.Direction.FORWARD);
        backleftDrive.setDirection(DcMotor.Direction.REVERSE);
        backrightDrive.setDirection(DcMotor.Direction.FORWARD);
        middleSlideDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleSlideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //imu
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );


        //Camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // does nothing if error
            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == tagOfInterest1) {
                        tagOfInterest = tag;
                        tagFound = true;
                        tagPosition = 1;
                        break;
                    }

                    if (tag.id == tagOfInterest2) {
                        tagOfInterest = tag;
                        tagFound = true;
                        tagPosition = 2;
                        break;
                    }

                    if (tag.id == tagOfInterest3) {
                        tagOfInterest = tag;
                        tagFound = true;
                        tagPosition = 3;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight at Position " + tagPosition);
                    imu.resetYaw();
                } else {
                    telemetry.addLine("Don't see tag of interest :(");
                }
            }
            telemetry.update();
            sleep(20);  // is required or else system will break
        }

        if (tagOfInterest != null) {
            telemetry.addLine("Tag was seen at position " + tagPosition);
            telemetry.addLine("Executing plan " + tagPosition);
            telemetry.update();

            if (tagPosition == 1) {
                frontleftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
                frontrightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
                backleftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
                backrightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
                middleslideDrive = hardwareMap.get(DcMotor.class, "middle_slides_drive");
                rightgripperDrive = hardwareMap.get(Servo.class, "right_gripper_drive");
                leftgripperDrive = hardwareMap.get(Servo.class, "left_gripper_drive");
                middleSlideDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                grippers(2, 500);
                setSlider(0.5, 2, 500);
                moveSimpleTargetPosition(.25, 500, 1, 100);
                moveSimpleTargetPosition(.25, 2700, 4, 100);
                moveSimpleTargetPosition(.25, 2250, 1, 100);
                moveSimpleTargetPosition(.25, 1000, 4, 100);;
                moveSimpleTargetPosition(.25, 200, 1, 100);
                setSlider(1, 3, 500);
                grippers(1, 500);
                moveSimpleTargetPosition(.25, 450, 3, 100);
                setSlider(0.5, 0, 500);



                moveSimpleTargetPosition(.25, 7000, 2, 500);

            }

            if (tagPosition == 2) {
                frontleftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
                frontrightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
                backleftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
                backrightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
                middleslideDrive = hardwareMap.get(DcMotor.class, "middle_slides_drive");
                rightgripperDrive = hardwareMap.get(Servo.class, "right_gripper_drive");
                leftgripperDrive = hardwareMap.get(Servo.class, "left_gripper_drive");
                middleSlideDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                grippers(2, 500);
                setSlider(0.5, 2, 500);
                moveSimpleTargetPosition(.25, 500, 1, 100);
                moveSimpleTargetPosition(.25, 2700, 4, 100);
                moveSimpleTargetPosition(.25, 2100, 1, 100);
                moveSimpleTargetPosition(.25, 1000, 4, 100);;
                moveSimpleTargetPosition(.25, 200, 1, 100);
                setSlider(1, 3, 500);
                grippers(1, 500);
                moveSimpleTargetPosition(.25, 450, 3, 100);
                setSlider(0.5, 0, 500);


                moveSimpleTargetPosition(.25, 1000, 2, 500);
                moveSimpleTargetPosition(.25, 200, 3, 500);
                moveSimpleTargetPosition(.25, 3000, 2, 500);
                moveSimpleTargetPosition(.25,600,1,500);
            }

            if (tagPosition == 3) {
                frontleftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
                frontrightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
                backleftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
                backrightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
                middleslideDrive = hardwareMap.get(DcMotor.class, "middle_slides_drive");
                rightgripperDrive = hardwareMap.get(Servo.class, "right_gripper_drive");
                leftgripperDrive = hardwareMap.get(Servo.class, "left_gripper_drive");
                middleSlideDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                grippers(2, 500);
                setSlider(0.5, 2, 500);
                moveSimpleTargetPosition(.25, 500, 1, 100);
                moveSimpleTargetPosition(.25, 2700, 4, 100);
                moveSimpleTargetPosition(.25, 2250, 1, 100);
                moveSimpleTargetPosition(.25, 1000, 4, 100);;
                moveSimpleTargetPosition(.25, 200, 1, 100);
                setSlider(1, 3, 500);
                grippers(1, 500);
                moveSimpleTargetPosition(.25, 450, 3, 100);
                setSlider(0.5, 0, 500);


                moveSimpleTargetPosition(.25, 1500, 2, 500);

            }

        } else {
            telemetry.addLine("No tag available, it was never seen during the init loop :(");
            telemetry.addLine("Backup plan INITIATED :)");
            while (!hasRun) {
            }
            telemetry.update();
        }
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

    public void grippers(int position, int sleep) {
        if (position == 1) {
            leftgripperDrive.setPosition(.77);
            rightgripperDrive.setPosition(.12);
        }

        if (position == 2) {
            leftgripperDrive.setPosition(.6);
            rightgripperDrive.setPosition(.4);
        }
        sleep(sleep);
    }

    public void setServo(double position) {
        position = position * 1;
        if (position == 1) {
            while (leftgripperDrive.getPosition() != .505) {
                leftgripperDrive.setPosition(.505);
                rightgripperDrive.setPosition(.35);
            }

        }
        if (position == 0) {
            while (leftgripperDrive.getPosition() != .77) {
                leftgripperDrive.setPosition(.77);
                rightgripperDrive.setPosition(.12);
            }

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
    }

    public void setSlider(double speed, double level, int sleep) {
        //middleslideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //middleslideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double position = 0;
        if (level == 0) {position = 0;}
        if (level == 1) {position = 1000;}
        if (level == 2) {position = 2600;}
        if (level == 3) {position = 4400;}
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
    public void turnGyro(double angle, double speed) {
        //insert method to reset gyro reading to 0
        boolean right = true;
        double currentAngle = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        if (currentAngle > angle) {right = true;}
        if (currentAngle < angle) {right = false;}
        if (right == true) {
            while (currentAngle > angle) {
                speed = (Math.abs(angle-currentAngle)/100)*speed;
                if (speed < 0.1) {speed = 0.1;}
                frontleftDrive.setPower(speed);
                backleftDrive.setPower(speed);
                frontrightDrive.setPower(-speed);
                backleftDrive.setPower(-speed);
            }
            if (currentAngle <= angle) {
                frontleftDrive.setPower(0);
                frontrightDrive.setPower(0);
                backleftDrive.setPower(0);
                backrightDrive.setPower(0);
            }
        }

    }


}