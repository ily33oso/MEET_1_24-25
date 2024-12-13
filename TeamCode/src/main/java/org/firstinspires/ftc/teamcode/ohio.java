package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import static java.lang.Thread.sleep;
@Autonomous(name="ohio", group="Robot")
public class ohio extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor Fr = null;
    private DcMotor Fl = null;
    private DcMotor Br = null;
    private DcMotor Bl = null;

    private DcMotor arm = null;
    private DcMotor slideL = null;
    private DcMotor slideR = null;
    private Servo cL, cR = null;
    private final ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 7.55906;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 1;

    static final double ARM_SPEED = 0.5;
    static final double LIFT_SPEED = 0.5;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        Fr = hardwareMap.get(DcMotor.class, "Fr");
        Fl = hardwareMap.get(DcMotor.class, "Fl");
        Bl = hardwareMap.get(DcMotor.class, "Br");
        Br = hardwareMap.get(DcMotor.class, "Bl");
        slideL = hardwareMap.get(DcMotor.class, "sL");
        slideR = hardwareMap.get(DcMotor.class, "sR");
        arm = hardwareMap.get(DcMotor.class, "a");
        cL = hardwareMap.get(Servo.class, "cL");
        cR = hardwareMap.get(Servo.class, "cR");

        //TEST
        Fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        Fr.setDirection(DcMotor.Direction.FORWARD);
        Fl.setDirection(DcMotor.Direction.REVERSE);
        Bl.setDirection(DcMotor.Direction.REVERSE);
        Br.setDirection(DcMotor.Direction.FORWARD);
        slideL.setDirection(DcMotor.Direction.FORWARD);
        slideR.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        cL.setDirection(Servo.Direction.FORWARD);
        cR.setDirection(Servo.Direction.FORWARD);

        Fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d :%7d :%7d :%7d :%7d",
                Fr.getCurrentPosition(),
                Fl.getCurrentPosition(),
                Bl.getCurrentPosition(),
                Br.getCurrentPosition(),
                slideL.getCurrentPosition(),
                slideR.getCurrentPosition(),
                arm.getCurrentPosition(),
                cL.getPosition(),
                cR.getPosition());
                telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //encoderarm(ARM_SPEED, 55, -55, 5.0);
        //encoderlift(LIFT_SPEED, 90, 90, 5.0);
        encoderDrive(DRIVE_SPEED, -50, 50, 5.0);  // S1: Forward 12 Inches with 5 Sec timeout
        encoderDrive(DRIVE_SPEED, -23, -23, 5.0);
        encoderDrive(DRIVE_SPEED, -15, 15, 5.0);
        //encoderarm(ARM_SPEED, -10, 10, 5.0);
        // TEMPORARY COMMENT:encoderStrafe(DRIVE_SPEED, 20, 20, 5.0);  // S2: Strafe Left 12 Inches with 4 Sec timeout
        // TEMPORARY COMMENT:encoderDrive(DRIVE_SPEED, 5, 5, 5.0); // S3: Reverse 10 Inches with 5 Sec timeout


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newFrTarget;
        int newFlTarget;
        int newBrTarget;
        int newBlTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newFrTarget = Fr.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newFlTarget = Fl.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newBrTarget = Bl.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newBlTarget = Br.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            Fr.setTargetPosition(newFrTarget);
            Fl.setTargetPosition(newFlTarget);
            Bl.setTargetPosition(newBrTarget);
            Br.setTargetPosition(newBlTarget);

            // Turn On RUN_TO_POSITION
            Fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            Fl.setPower(Math.abs(speed));
            Fr.setPower(Math.abs(speed));
            Br.setPower(Math.abs(speed));
            Bl.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (Fl.isBusy() && Br.isBusy() && Bl.isBusy() && Fr.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d :%7d :%7d", newFlTarget, newFrTarget, newBlTarget, newBrTarget);
                telemetry.addData("Currently at", " at %7d :%7d :%7d :%7d", newFlTarget, newFrTarget, newBlTarget, newBrTarget,
                        Fl.getCurrentPosition(), Fr.getCurrentPosition(), Br.getCurrentPosition(), Bl.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            Fl.setPower(0);
            Fr.setPower(0);
            Br.setPower(0);
            Bl.setPower(0);

            // Turn off RUN_TO_POSITION
            Fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
//Arm encoders

    public void encoderarm (double speed, double fowardInches, double reverseInches, double timeoutS) {
        int newarmTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newarmTarget = arm.getCurrentPosition() + (int)(fowardInches * COUNTS_PER_INCH);
            arm.setTargetPosition(newarmTarget);


            // Turn On RUN_TO_POSITION
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            arm.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (arm.isBusy())){

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newarmTarget);
                telemetry.addData("Currently at",  " at %7d :%7d", newarmTarget, arm.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            arm.setPower(0);

            // Turn off RUN_TO_POSITION
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    //lift encoder

    public void encoderlift(double speed, double fowardInches, double reverseInches, double timeoutS) {
        int newslideLTarget;
        int newslidesRTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newslideLTarget = slideL.getCurrentPosition() + (int)(fowardInches * COUNTS_PER_INCH);
            newslidesRTarget = slideR.getCurrentPosition() + (int)(fowardInches*COUNTS_PER_INCH);

            slideR.setTargetPosition(newslidesRTarget);
            slideL.setTargetPosition(newslideLTarget);


            // Turn On RUN_TO_POSITION
            slideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            slideL.setPower(Math.abs(speed));
            slideR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (slideL.isBusy()) && (slideR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d", newslideLTarget, newslidesRTarget);
                telemetry.addData("Currently at",  " at %7d", newslideLTarget, newslidesRTarget, slideL.getCurrentPosition(), slideR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            slideL.setPower(0);
            slideR.setPower(0);

            // Turn off RUN_TO_POSITION
            slideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    //strafing encoders
    /*
    public void encoderStrafe(double speed, double leftInches, double rightInches, double timeoutS){

        int newFRTarget;
        int newFLTarget;
        int newBRTarget;
        int newBLTarget;

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            // For strafing one side's wheels 'attract' each other while the other side 'repels' each other
            //The positive value will make the robot go to the left with current arrangement
            newFRTarget = FR.getCurrentPosition() - (int)(rightInches * COUNTS_PER_INCH);
            newFLTarget = FL.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBRTarget = BR.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newBLTarget = BL.getCurrentPosition() - (int)(leftInches * COUNTS_PER_INCH);
            FR.setTargetPosition(newFRTarget);
            FL.setTargetPosition(newFLTarget);
            BR.setTargetPosition(newBRTarget);
            BL.setTargetPosition(newBLTarget);

            // Turn On RUN_TO_POSITION
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            FL.setPower(Math.abs(speed));
            FR.setPower(Math.abs(speed));
            BL.setPower(Math.abs(speed));
            BR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (FL.isBusy() && BL.isBusy() && BR.isBusy() && FR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newFLTarget,  newFRTarget, newBLTarget, newBRTarget);
                telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d", newFLTarget, newFRTarget, newBLTarget, newBRTarget,
                        FL.getCurrentPosition(), FR.getCurrentPosition(), BL.getCurrentPosition(), BR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);

            // Turn off RUN_TO_POSITION
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
     */
    //private boolean opModeIsActive() {
    //}
}