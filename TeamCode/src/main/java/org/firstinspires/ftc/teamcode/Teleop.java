package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="Teleop", group="Linear Opmode")

public class Teleop extends LinearOpMode {

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


    // @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // step (using the FTC Robot Controller app on the phone).
        frontleftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontrightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backleftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backrightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        middleslideDrive= hardwareMap.get(DcMotor.class, "middle_slides_drive");
        rightgripperDrive = hardwareMap.get(Servo.class, "right_gripper_drive");
        leftgripperDrive = hardwareMap.get(Servo.class, "left_gripper_drive");

        // Define Boolean Buttons
        boolean aPress1;
        boolean bPress1;
        boolean xPress1;
        boolean yPress1;
        boolean dpad_up2 = false;
        boolean dpad_down2 = false;
        boolean rBPress2;
        boolean lBPress2;
        boolean rBPress1;
        boolean lBPress1;
        boolean fieldCentric = true;


        // Define Movement Variables
        double power = .5;
        double vertical;
        double horizontal;
        double pivot;
        double driveTurn = 0;
        double gamepadXCoordinate;
        double gamepadYCoordinate;
        double gamepadHypot;
        double gamepadDegree = 0;
        double robotDegree = 0;
        double movementDegree;
        double gamepadXControl = 0;
        double gamepadYControl = 0;

        // Define Motors & starts intial values
        frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleslideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleslideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontleftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontrightDrive.setDirection(DcMotor.Direction.REVERSE);
        backleftDrive.setDirection(DcMotor.Direction.FORWARD);
        backrightDrive.setDirection(DcMotor.Direction.REVERSE);
        middleslideDrive.setDirection(DcMotor.Direction.FORWARD);

        frontleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        middleslideDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set up IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
        );

        // Wait for the game to start (driver presses PLAY)
        imu.initialize(myIMUparameters);
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Assign Variables to buttons
            dpad_up2 = gamepad2.dpad_up;
            dpad_down2 = gamepad2.dpad_down;

            rBPress2 = gamepad2.right_bumper;
            lBPress2 = gamepad2.left_bumper;
            rBPress1 = gamepad1.right_bumper;
            lBPress1 = gamepad1.left_bumper;

            aPress1 = gamepad1.a;
            bPress1 = gamepad1.b;
            yPress1 = gamepad1.y;
            xPress1 = gamepad1.x;

            //slides code
            if (dpad_up2 && middleslideDrive.getCurrentPosition() <= 5000) {

                if (middleslideDrive.getCurrentPosition() <= 5000) {
                    middleslideDrive.setPower(-1);
                }
            }

            if (dpad_down2 & middleslideDrive.getCurrentPosition() >= 0) {
                if (middleslideDrive.getCurrentPosition() >= 0) {
                    middleslideDrive.setPower(1);
                }
            }

            if (!dpad_down2 && !dpad_up2) {
                middleslideDrive.setPower(0);
            }
            // Controls grippers
            if (lBPress2) {
                leftgripperDrive.setPosition(.77);
                rightgripperDrive.setPosition(.12);
            }
            if (rBPress2) {
                leftgripperDrive.setPosition(.505);
                rightgripperDrive.setPosition(.35);
            }

            // Moves robot by using joystick position
            // A and B button changes power
            if (yPress1) {power = 1; }
            else if (aPress1) { power = .5; }
            else if (xPress1) { fieldCentric = true; }
            else if (bPress1) { fieldCentric = false; }

            if (!fieldCentric) {
                vertical = -gamepad1.left_stick_y;
                horizontal = gamepad1.left_stick_x;
                pivot = gamepad1.right_stick_x;

                frontrightDrive.setPower(power * (pivot + (-vertical + horizontal)));
                backrightDrive.setPower(power * (pivot + (-vertical - horizontal)));
                frontleftDrive.setPower(power * (-pivot + (-vertical - horizontal)));
                backleftDrive.setPower(power * (-pivot + (-vertical + horizontal)));

                if (lBPress1 && rBPress1) {
                    frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    middleslideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    middleslideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

            } else if (fieldCentric) {
                robotDegree = getAngle();

                driveTurn = gamepad1.right_stick_x;
                gamepadXCoordinate = gamepad1.left_stick_x;
                gamepadYCoordinate = -gamepad1.left_stick_y;

                gamepadHypot = Range.clip(Math.hypot(gamepadXCoordinate, gamepadYCoordinate), 0, 1);
                gamepadDegree = Math.atan2(gamepadYCoordinate, gamepadXCoordinate);

                movementDegree = gamepadDegree - robotDegree;

                gamepadXControl = Math.cos(movementDegree) * gamepadHypot;
                gamepadYControl = Math.sin(movementDegree) * gamepadHypot;

                frontrightDrive.setPower(power * (driveTurn + (-gamepadYControl + gamepadXControl)));
                backrightDrive.setPower(power * (driveTurn + (-gamepadYControl - gamepadXControl)));
                frontleftDrive.setPower(power * (-driveTurn + (-gamepadYControl - gamepadXControl)));
                backleftDrive.setPower(power * (-driveTurn + (-gamepadYControl + gamepadXControl)));

                if (lBPress1 && rBPress1) {
                    imu.resetYaw();
                }
            }


        // Telemetry
        telemetry.addData("Run Time", runtime.toString());
        telemetry.addData("Front Right Encoder", frontrightDrive.getCurrentPosition());
        telemetry.addData("Front Left Encoder", frontleftDrive.getCurrentPosition());
        telemetry.addData("Back Right Encoder", backrightDrive.getCurrentPosition());
        telemetry.addData("Back Left Encoder", backleftDrive.getCurrentPosition());
            telemetry.addData("Middle Slides Encoder", middleslideDrive.getCurrentPosition());
        telemetry.addData("IMU Z Angle", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("robo degree", robotDegree);
        telemetry.addData("X", gamepadXControl);
        telemetry.addData("Y", gamepadYControl);
        telemetry.addData("Turn", driveTurn);
        telemetry.addData("Gamepad", gamepadDegree);
        telemetry.addData("Robo", robotDegree);
        telemetry.update();
    }
}

    public double getAngle() {
        return imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }
}