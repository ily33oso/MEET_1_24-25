package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class RealTele extends LinearOpMode {
    static final double COUNTS_PER_MOTOR_REV = 375 ; //EG:MOTOR ENCODER
    static final double DRIVE_GEAR_REDUCTION = 0.5 ; //NO EXTERNAL GEARING
    static final double WHEEL_DIAMETER_INCHES = 7.55906 ; // FOR FIGURING CIRCUMFERENCES
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("Fl");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("Bl");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("Fr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("Br");
        DcMotor slideL = hardwareMap.dcMotor.get("sL");
        DcMotor slideR = hardwareMap.dcMotor.get("sR");
        DcMotor arm = hardwareMap.dcMotor.get("a");
        Servo clawL = hardwareMap.servo.get("cL");
        Servo clawR = hardwareMap.servo.get("cR");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideR.setDirection(DcMotorSimple.Direction.REVERSE);
        slideL.setDirection(DcMotorSimple.Direction.FORWARD);
        clawR.setDirection(Servo.Direction.REVERSE);
        clawL.setDirection(Servo.Direction.FORWARD);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
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
            double frontLeftPower = (rotY - rotX + rx) / denominator;
            double backLeftPower = (rotY + rotX + rx) / denominator;
            double frontRightPower = (rotY + rotX - rx) / denominator;
            double backRightPower = (rotY - rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if (gamepad2.a) {
                clawL.setPosition(0.35);
                clawR.setPosition(0.35);
            } else {
                clawL.setPosition(0);
                clawR.setPosition(0);
            }
            if (gamepad2.dpad_up) {
                slideL.setPower(-1);
                slideR.setPower(-1);
            } else if (gamepad2.dpad_down) {
                slideL.setPower(1);
                slideR.setPower(1);
            } else {
                slideL.setPower(0);
                slideR.setPower(0);
            }
            if (gamepad2.dpad_right) {
                arm.setPower(.3);
            } else if (gamepad2.dpad_left) {
                arm.setPower(-.3);
            } else {
                arm.setPower(0);
            }
            telemetry.addData("Lift Position",slideL.getCurrentPosition() + slideR.getCurrentPosition() + (int)(COUNTS_PER_INCH) );
        }
    }
}
