package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class autoFoward extends LinearOpMode {
    DcMotor Fr, Fl, Bl, Br, arm, slideL, slideR;
    Servo cL, cR;
    @Override
    public void runOpMode() throws InterruptedException {
        {
            Fr = hardwareMap.dcMotor.get("Fr");
            Fl = hardwareMap.dcMotor.get("Fl");
            Br = hardwareMap.dcMotor.get("Br");
            Bl = hardwareMap.dcMotor.get("Bl");
            slideL = hardwareMap.dcMotor.get("sL");
            slideR = hardwareMap.dcMotor.get("sR");
            arm = hardwareMap.dcMotor.get("a");

            cL = hardwareMap.servo.get("cL");
            cR = hardwareMap.servo.get("cR");

            Fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            Fr.setDirection(DcMotorSimple.Direction.FORWARD);
            Br.setDirection(DcMotorSimple.Direction.FORWARD);
            Fl.setDirection(DcMotorSimple.Direction.REVERSE);
            Bl.setDirection(DcMotorSimple.Direction.REVERSE);
            slideR.setDirection(DcMotorSimple.Direction.REVERSE);
            slideL.setDirection(DcMotorSimple.Direction.FORWARD);
            cR.setDirection(Servo.Direction.REVERSE);
            cL.setDirection(Servo.Direction.FORWARD);

            waitForStart();
            Fr.setPower(0.5);
            Fl.setPower(-0.45);
            Br.setPower(-0.5);
            Bl.setPower(0.45);

            telemetry.addData("Path", "Complete");
            telemetry.update();;
            sleep(1100);
        }
    }
}