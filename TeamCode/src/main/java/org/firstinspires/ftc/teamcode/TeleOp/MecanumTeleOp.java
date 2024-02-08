package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.CountDownTimer;
import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.Hardware.MecanumDriveHardware;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Mecanum Base", group = "TeleOp")
public class MecanumTeleOp extends OpMode { // Coach Review: I prefer to use LinearOpMode always. Just FYI.

    MecanumDriveHardware robot = new MecanumDriveHardware();
    ImprovedGamepad impGamepad1;
    ImprovedGamepad impGamepad2;
//    CountDownTimer timer = new CountDownTimer(ElapsedTime.Resolution.SECONDS);
    ElapsedTime outtakeTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public double rightStickYVal;
    public double rightStickXVal;
    public double leftStickYVal;
    public double leftStickXVal;
    public double rotate;
    public double speedMod = 0.9;
    public double turnMod = 0.9;
    public double liftMod = 1;

//    public double maxLiftDistance = 7500;
//    public double minLiftDistance = -7500;

    public double startFrontLeft;

    public boolean slowMode = false;

    public boolean outtakeOpen = false;

//    public boolean launcherOn = false;

    @Override
    public void init() {
        impGamepad1 = new ImprovedGamepad(gamepad1, new ElapsedTime(), "gamepad1");
        impGamepad2 = new ImprovedGamepad(gamepad2, new ElapsedTime(), "gamepad2");
        robot.init(this.hardwareMap, telemetry);

        // Coach Review: with STOP_AND_RESET_ENCODERS inside of robot.ini(), this should always be 0, right?
        startFrontLeft = robot.frontLeft.getCurrentPosition();

//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Coach Review: OuttakeTimer is unused, and startTime doesn't do what you think.
        outtakeTimer.startTime();

//        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        impGamepad1.update();
        impGamepad2.update();

        leftStickXVal = impGamepad1.left_stick_x.getValue() * speedMod;
        leftStickYVal = impGamepad1.left_stick_y.getValue() * speedMod;
        rightStickXVal = impGamepad1.right_stick_x.getValue() * speedMod;
        rightStickYVal = impGamepad1.right_stick_y.getValue() * speedMod;

        rotate = rightStickXVal * turnMod;

        // Coach Review: Most teams standardize on the GM0 controls for mecanum driving.
        // https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
        // In particular Y stick up is -1.0, and down is +1.0.
        // This can be compensated for in any number of ways, such as reversing the motors in your
        // hardware class.
        // Feel free to ignore this comment -- I only add it for completeness. If what you have is
        // working for your driver, that is fine.
        robot.backLeft.setPower(leftStickYVal+leftStickXVal+rotate);
        robot.frontLeft.setPower(leftStickYVal-leftStickXVal+rotate);
        robot.backRight.setPower(leftStickYVal-leftStickXVal-rotate);
        robot.frontRight.setPower(leftStickYVal+leftStickXVal-rotate);

//        robot.frontLeft.setPower(leftStickYVal);
//        robot.frontRight.setPower(leftStickYVal);

        // Coach Review: What is the rotation? This seems to be how far the front left motor has
        // moved overall, not just rotating?
        telemetry.addData("The ROTATION:", startFrontLeft - robot.frontLeft.getCurrentPosition());


/*
        // preset positions for lift(not wanted currently)
        if(impGamepad1.y.isInitialPress()){
            robot.lift.setTargetPosition(-7400);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(1 * liftMod);
            while(robot.lift.isBusy()){}
            robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if(impGamepad1.a.isInitialPress()){
            robot.lift.setTargetPosition(-100);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(1 * liftMod);
            while(robot.lift.isBusy()){}
            robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


 */
//moves slide up and down with left and right triggers.
        if (impGamepad1.left_trigger.getValue() > 0.02) {
            robot.lift.setPower(-impGamepad1.left_trigger.getValue() * liftMod);
        } else if(impGamepad1.right_trigger.getValue() > 0.02){
            robot.lift.setPower(impGamepad1.right_trigger.getValue() * liftMod);
        } else {
            robot.lift.setPower(0);
        }
        telemetry.addData("Controller 1 R Trigger", impGamepad1.right_trigger.getValue());
        telemetry.addData("Controller 1 L Trigger", impGamepad1.left_trigger.getValue());
        telemetry.addData("Lift Power", robot.lift.getPower());

        // setting max and min for the lift so the robot doesn't break, manual
//        if(robot.lift.getCurrentPosition() > -7500 && impGamepad1.right_trigger.getValue() != 0) {
//            if (impGamepad1.right_trigger.getValue() > 0) {
//                robot.lift.setPower(-impGamepad1.right_trigger.getValue() * liftMod);
//            } else {
//                robot.lift.setPower(0);
//            }
//        } else if(robot.lift.getCurrentPosition() <= -100) {
//            if (impGamepad1.left_trigger.getValue() > 0) {
//                robot.lift.setPower(impGamepad1.left_trigger.getValue() * liftMod);
//            } else {
//                robot.lift.setPower(0);
//            }
//        } else {
//            robot.lift.setPower(0);
//        }

        //intake is controlled by driver1's dpad
        if(impGamepad1.dpad_up.isInitialPress()){
            robot.intake.setPower(0.4);
        }else if(impGamepad1.dpad_left.isInitialPress()){
            robot.intake.setPower(0);
        }else if(impGamepad1.dpad_down.isInitialPress()){
            robot.intake.setPower(-0.4);
        }

        //changed speed ut not uploaded to robot

        if(impGamepad1.left_bumper.isInitialPress() || impGamepad1.right_bumper.isInitialPress()) {
            robot.planeServo.setPosition(0);
        }

        if(impGamepad1.x.isInitialPress()){
            if(!outtakeOpen){
                robot.outtake.setPosition(0.35);
                outtakeOpen = true;
            } else {
                robot.outtake.setPosition(0.01);
                outtakeOpen = false;
            }
        }

        if(impGamepad1.a.isInitialPress()){
            robot.transfer.setPower(1);
        } else if(impGamepad1.b.isInitialPress()){
            robot.transfer.setPower(0);
        } else if (impGamepad1.y.isInitialPress()){
            robot.transfer.setPower(-1);
        }

        if(impGamepad1.dpad_right.isInitialPress()){
            if(slowMode == false) {
                speedMod = 0.5;
                slowMode = true;
            } else if(slowMode == true){
                speedMod = 0.9;
                slowMode = false;
            }
        }

        telemetry.addData("Lift Position:", robot.lift.getCurrentPosition());
        telemetry.addData("Outtake Position", robot.outtake.getPosition());
        telemetry.update(); // Coach Review: update() is not needed from OpMode and loop()
    }
}