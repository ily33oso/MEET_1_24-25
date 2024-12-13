package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleOp2 extends OpMode{
    static final double COUNTS_PER_MOTOR_REV = 435 ; //EG:MOTOR ENCODER
    static final double DRIVE_GEAR_REDUCTION = 0.5 ; //NO EXTERNAL GEARING
    static final double WHEEL_DIAMETER_INCHES = 7.55906 ; // FOR FIGURING CIRCUMFERENCES
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    DcMotor Fl, Bl, Fr, Br, Lslide, Rslide, arm;
    Servo Lclaw, Rclaw;

    @Override
    public void  init(){
        Br = hardwareMap.dcMotor.get("Br");
        Fr = hardwareMap.dcMotor.get("Fr");
        Fl = hardwareMap.dcMotor.get("Fl");
        Bl = hardwareMap.dcMotor.get("Bl");
        arm = hardwareMap.dcMotor.get("arm");
        Lslide = hardwareMap.dcMotor.get("Lslide");
        Rslide = hardwareMap.dcMotor.get("Rslide");
        Lclaw = hardwareMap.servo.get("Lclaw");
        Rclaw = hardwareMap.servo.get("Rclaw");
    }
    @Override
    public void loop() {
        //lift encoders
        // base movements
        if (Math.abs(gamepad1.right_stick_y)> .2){
            Fr.setPower(gamepad1.right_stick_y * 1);
            Br.setPower(gamepad1.right_stick_y * 1);
        } else {
            Br.setPower(0);
            Fr.setPower(0);
        }
        if (Math.abs(gamepad1.left_stick_y)> .2){
            Fl.setPower(gamepad1.left_stick_y * -1);
            Bl.setPower(gamepad1.left_stick_y * 1);
        } else {
            Fl.setPower(0);
            Bl.setPower(0);
        }
        if (gamepad1.right_bumper) {
            Fr.setPower(1);
            Br.setPower(1);
            Fl.setPower(1);
            Bl.setPower(1);
        } else {
            Fr.setPower(0);
            Br.setPower(0);
            Fl.setPower(0);
            Bl.setPower(0);
        }
        if (gamepad1.left_bumper) {
            Fr.setPower(-1);
            Br.setPower(-1);
            Fl.setPower(-1);
            Bl.setPower(-1);
        } else {
            Fr.setPower(0);
            Br.setPower(0);
            Fl.setPower(0);
            Bl.setPower(0);
        }
        // arm
        if (Math.abs(gamepad2.right_stick_y) > .2) {
            arm.setPower(gamepad2.right_stick_y * -0.4);
        } else {
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setPower(0);
        }
        //if the top part doesnt work try this
        /*
        if (gamepad2.left_bumper) {
            arm.setPower(0.5);
        } else if (gamepad2.right_bumper) {
            arm.setPower(-0.5);
        } else {
            arm.setPower(0);
        }
        */

        // CLAW
        // check to see how they are
            if (gamepad1.b) {
                Lclaw.setPosition(10);
                Rclaw.setPosition(-10);
            } else if (gamepad2.b) {
                Lclaw.setPosition(-10);
                Rclaw.setPosition(10);
            } else {
                Lclaw.setPosition(0);
                Rclaw.setPosition(0);
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
    telemetry.addData("Lift Position",Lslide.getCurrentPosition() + Rslide.getCurrentPosition() + (int)(COUNTS_PER_INCH) );

    }
}
