package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.Hardware.MecanumDriveHardware;

@TeleOp(name = "Mecanum Base", group = "TeleOp")
public class MecanumTeleOp extends OpMode {

    MecanumDriveHardware robot = new MecanumDriveHardware();
    ImprovedGamepad impGamepad1;
    ImprovedGamepad impGamepad2;

    public double rightStickYVal;
    public double rightStickXVal;
    public double leftStickYVal;
    public double leftStickXVal;
    public double rotate;
    public double speedMod = 0.5;
    public double turnMod = 0.6;
    public double liftMod = 1;

    public double maxLiftDistance = 7500;

    public double minLiftDistance = -7500;

    public double intakeTogglePower = 0;
    public boolean launcherOn = false;


    @Override
    public void init() {
        impGamepad1 = new ImprovedGamepad(gamepad1, new ElapsedTime(), "gamepad1");
        impGamepad2 = new ImprovedGamepad(gamepad2, new ElapsedTime(), "gamepad2");
        robot.init(this.hardwareMap, telemetry);

//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        impGamepad1.update();
        impGamepad2.update(); // Rachel -- this was missing, so controller 2 wasn't doing anything

        leftStickXVal = impGamepad1.left_stick_x.getValue() * speedMod;
        leftStickYVal = impGamepad1.left_stick_y.getValue() * speedMod;
        rightStickXVal = impGamepad1.right_stick_x.getValue() * speedMod;
        rightStickYVal = impGamepad1.right_stick_y.getValue() * speedMod;

        rotate = rightStickXVal * turnMod;

        robot.backLeft.setPower(leftStickYVal-leftStickXVal+rotate);
        robot.frontLeft.setPower(leftStickYVal+leftStickXVal+rotate);
        robot.backRight.setPower(leftStickYVal+leftStickXVal-rotate);
        robot.frontRight.setPower(leftStickYVal-leftStickXVal-rotate);
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
        if (impGamepad1.left_trigger.getValue() > 0.05 && robot.lift.getCurrentPosition() > minLiftDistance) {
            robot.lift.setPower(-impGamepad1.left_trigger.getValue() * liftMod);
        } else if(impGamepad1.right_trigger.getValue() > 0.05 && robot.lift.getCurrentPosition() < maxLiftDistance){
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

   /*     // Intake is controlled by driver2 from the trigger
        if(impGamepad2.right_trigger.getValue() > 0){
            robot.intake.setPower(-impGamepad2.right_trigger.getValue());
        } else if(impGamepad2.left_trigger.getValue() > 0){
            robot.intake.setPower(impGamepad2.left_trigger.getValue());
        } else {
            robot.intake.setPower(0);
        }
        telemetry.addData("Controller 2 R Trigger", impGamepad2.right_trigger.getValue());
        telemetry.addData("Controller 2 L Trigger", impGamepad2.left_trigger.getValue());


    */

        //intake is controlled by driver1's dpad
        if(impGamepad1.dpad_up.isInitialPress()){
            intakeTogglePower = 1;
        }else if(impGamepad1.dpad_left.isInitialPress()){
            intakeTogglePower = 0;
        }else if(impGamepad1.dpad_down.isInitialPress()){
            intakeTogglePower = -1;
        }
        robot.intake.setPower(intakeTogglePower);

        // Let the driver control the intake too -- but with the bumper
//        if(impGamepad1.right_bumper.isPressed()){
//            robot.intake.setPower(0.3);
//        } else if(impGamepad1.left_bumper.isPressed()){
//            robot.intake.setPower(-0.3);
//        }
/*
        launcherOn = false;
        if(impGamepad2.right_bumper.isInitialPress()){
            if(!launcherOn){
                robot.launcher.setPower(0.9);
                launcherOn = true;
            } else if(launcherOn){
                robot.launcher.setPower(0);
                launcherOn = false;
            }
        }

 */

        if(impGamepad1.right_bumper.isInitialPress() && !launcherOn){
            launcherOn = true;
            robot.launcher.setPower(0.9);
        } else if (impGamepad1.right_bumper.isInitialPress() && launcherOn) {
            launcherOn = false;
            robot.launcher.setPower(0);
        }

        if(impGamepad1.dpad_right.isInitialPress()){
            robot.launcher.setPower(-0.9);
        } else if (!impGamepad1.dpad_right.isInitialPress() && !launcherOn) {
            robot.launcher.setPower(0);
        }

        if(impGamepad1.left_bumper.isPressed()) {
            robot.planeServo.setPosition(0.75);
        }

        if(impGamepad1.x.isInitialPress()){
            robot.outtake.setPosition(0);
        } else if(impGamepad1.a.isInitialPress()){
            robot.outtake.setPosition(0.5);
        }

        telemetry.addData("Lift Position:", robot.lift.getCurrentPosition());
        telemetry.update();
    }
}