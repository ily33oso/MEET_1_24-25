package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FieldCentricTeley extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 384.5;    // eg: TETRIX Motor Encoder

    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.

    static final double WHEEL_DIAMETER_INCHES = 3.77952;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    boolean slowMode = false;


    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor Fl = hardwareMap.dcMotor.get("Fl");
        DcMotor Bl = hardwareMap.dcMotor.get("Bl");
        DcMotor Fr = hardwareMap.dcMotor.get("Fr");
        DcMotor Br = hardwareMap.dcMotor.get("Br");



        Servo Lclaw = hardwareMap.servo.get("Lclaw");
        Servo Rclaw = hardwareMap.servo.get("Rclaw");



        DcMotor arm = hardwareMap.dcMotor.get("arm");
        DcMotor Lslide = hardwareMap.dcMotor.get("Lslide");
        DcMotor Rslide = hardwareMap.dcMotor.get("Rslide");


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        Br.setDirection(DcMotor.Direction.REVERSE);
        Fr.setDirection(DcMotor.Direction.REVERSE);
        Fl.setDirection(DcMotor.Direction.FORWARD);
        Bl.setDirection(DcMotor.Direction.FORWARD);




        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {



            double triggerVal = gamepad1.left_trigger;
            double powerFactor = slowMode ? 0.2 : 0.5;


            if (triggerVal > 0.1) {
                // Map trigger value from [0.1, 1] to [0.5, 1] for finer control
                powerFactor = 0.2 + 0.2 * (1 - triggerVal);
            }



            // lift without a limiter
            if (gamepad2.dpad_up) {
                Lslide.setPower(1);
                Rslide.setPower(1);
            } else if (gamepad2.dpad_down) {
                Lslide.setPower(-1);
                Rslide.setPower(-1);
            } else {
                Lslide.setPower(0);
                Rslide.setPower(0);
            }


            if (Math.abs(gamepad2.right_stick_y) > .2) {
                arm.setPower(gamepad2.right_stick_y * -0.4);
            } else {
                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                arm.setPower(0);
            }
            // CLAW
            // check to see how they are
            if (gamepad1.b) {
                Lclaw.setPosition(1);
                Rclaw.setPosition(-1);
            } else if (gamepad2.b) {
                Lclaw.setPosition(-1);
                Rclaw.setPosition(1);
            } else {
                Lclaw.setPosition(0);
                Rclaw.setPosition(0);
            }

            //arm code
            if (gamepad2.left_bumper) {
            arm.setPower(0.5);
            } else if (gamepad2.right_bumper) {
            arm.setPower(-0.5);
            } else {
            arm.setPower(0);
            }



            /* eli's code
            if (Math.abs(gamepad2.right_stick_y) > .1){
                arm.setPower(gamepad2.right_stick_y * 0.7);
            } else{
                arm.setPower(0);
            }

    `       */

            double y = -gamepad1.left_stick_x ; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_y ;
            double rx = gamepad1.right_stick_x ;
            y *= powerFactor;
            x *= powerFactor;
            rx *= powerFactor;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.start) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            Fl.setPower(frontLeftPower);
            Bl.setPower(backLeftPower);
            Fr.setPower(frontRightPower);
            Br.setPower(backRightPower);

            //maybe need not too sure ;-;
            telemetry.addData("Lift Position",Lslide.getCurrentPosition() + Rslide.getCurrentPosition() + (int)(COUNTS_PER_INCH) );


        }
    }



}
