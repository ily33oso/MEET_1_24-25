package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "GATO2")
public class GATO2 extends OpMode {


    DcMotor Fr;
    DcMotor Br;
    DcMotor Fl;
    DcMotor Bl;


    @Override
    public void init() {
        Fr = hardwareMap.dcMotor.get("Fr");
        Br = hardwareMap.dcMotor.get("Br");
        Fl = hardwareMap.dcMotor.get("Fl");
        Bl = hardwareMap.dcMotor.get("Bl");
    }


    @Override
    public void loop() {
        //Front back Right
        if (Math.abs(-gamepad1.right_stick_y) > .2) {
            Fr.setPower(-gamepad1.right_stick_y * 1);
            Br.setPower(-gamepad1.right_stick_y * 1);
        } else {
            Br.setPower(0);
            Fr.setPower(0);
        }
        if (Math.abs(-gamepad1.left_stick_y) > .2) {
            Fl.setPower(-gamepad1.left_stick_y * 1);
            Bl.setPower(-gamepad1.left_stick_y * 1);
        } else {
            Fl.setPower(0);
            Bl.setPower(0);
        }
        if (Math.abs(-gamepad1.right_trigger) > .2) {
            Fl.setPower(0.8);
            Br.setPower(-0.8);
            Bl.setPower(-0.8);
            Fr.setPower(0.8);
        } else {
            Fr.setPower(0);
            Bl.setPower(0);
            Br.setPower(0);
            Fl.setPower(0);
        }
        if (Math.abs(-gamepad1.left_trigger) > .2) {
            Fl.setPower(-0.8);
            Br.setPower(0.8);
            Bl.setPower(0.8);
            Fr.setPower(-0.8);
        } else {
            Fr.setPower(0);
            Bl.setPower(0);
            Br.setPower(0);
            Fl.setPower(0);
        }
    }
}

