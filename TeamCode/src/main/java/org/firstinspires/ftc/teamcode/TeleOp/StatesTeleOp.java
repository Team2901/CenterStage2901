package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.StatesHardware;
import org.firstinspires.ftc.teamcode.Utilities.CountDownTimer;
import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.Hardware.MecanumDriveHardware;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "States Mecanum Base", group = "TeleOp")
public class StatesTeleOp extends OpMode {

    StatesHardware robot = new StatesHardware();
    ImprovedGamepad impGamepad1;
    ImprovedGamepad impGamepad2;
    ElapsedTime outtakeTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public double rightStickYVal;
    public double rightStickXVal;
    public double leftStickYVal;
    public double leftStickXVal;
    public double rotate;
    public double speedMod = 0.9;
    public double turnMod = 0.9;

    public int maxArmTicks = 7500;
    public int currentArmTicks = 0;

    public double startFrontLeft;

    public boolean slowMode = false;

    public boolean outtakeRightClosed = false;
    public boolean outtakeLeftClosed = false;

    double armAngle = 45.0;
    double error = 0.0;
    double total = 0.0;
    double kp = 0.9;
    double ki = 0.0;
    double kd = 0.0;
    double kCos = 0.3;
    double pArm = 0.0;
    double iArm = 0.0;
    double dArm = 0.0;
    double cosArm = 0.0;
    double iArmMax = 0.25;
    @Override
    public void init() {
        impGamepad1 = new ImprovedGamepad(gamepad1, new ElapsedTime(), "gamepad1");
        impGamepad2 = new ImprovedGamepad(gamepad2, new ElapsedTime(), "gamepad2");
        robot.init(this.hardwareMap, telemetry);

        startFrontLeft = robot.frontLeft.getCurrentPosition();

        outtakeTimer.startTime();
    }

    @Override
    public void loop() {
        impGamepad1.update();
        impGamepad2.update();

        //basic drive base stuff
        leftStickXVal = impGamepad1.left_stick_x.getValue() * speedMod;
        leftStickYVal = impGamepad1.left_stick_y.getValue() * speedMod;
        rightStickXVal = impGamepad1.right_stick_x.getValue() * speedMod;
        rightStickYVal = impGamepad1.right_stick_y.getValue() * speedMod;

        rotate = rightStickXVal * turnMod;

        robot.backLeft.setPower(leftStickYVal+leftStickXVal+rotate);
        robot.frontLeft.setPower(leftStickYVal-leftStickXVal+rotate);
        robot.backRight.setPower(leftStickYVal-leftStickXVal-rotate);
        robot.frontRight.setPower(leftStickYVal+leftStickXVal-rotate);

        telemetry.addData("The ROTATION:", startFrontLeft - robot.frontLeft.getCurrentPosition());

        //might have to use PID for the arm, we'll see

        //set arm to max height
        if(impGamepad1.dpad_up.isInitialPress()){
//            robot.arm.setTargetPosition(maxArmTicks);
//            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.arm.setPower(0.6); //negatives might have to be flipped
//            while(robot.arm.isBusy()){}
//            robot.arm.setPower(0);
//            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //armPower(maxArmTicks);
        }

        //set arm to min height/ground
        if(impGamepad1.dpad_down.isInitialPress()){
//            robot.arm.setTargetPosition(0);
//            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.arm.setPower(-0.6); //negatives might have to be flipped
//            while(robot.arm.isBusy()){}
//            robot.arm.setPower(0);
//            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //armPower(0);
        }

        //arm up
        if(impGamepad1.right_trigger.getValue() > 0){
            robot.arm.setPower(impGamepad1.right_trigger.getValue());
            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            currentArmTicks = robot.arm.getCurrentPosition();
        } else if(impGamepad1.left_trigger.getValue() > 0){
            robot.arm.setPower(-impGamepad1.left_trigger.getValue());
            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            currentArmTicks = robot.arm.getCurrentPosition();
        } else {
            robot.arm.setTargetPosition(currentArmTicks);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm.setPower(1);
        }

        //right claw toggle
        if(impGamepad1.right_bumper.isInitialPress()){
            if(!outtakeRightClosed) {
                robot.outtakeRight.setPosition(1); //update new servo positions
                outtakeRightClosed = true;
            } else {
                robot.outtakeRight.setPosition(0); //update new servo positions
                outtakeRightClosed = false;
            }
        }

        //left claw toggle
        if(impGamepad1.left_bumper.isInitialPress()){
            if(!outtakeLeftClosed) {
                robot.outtakeLeft.setPosition(1); //update new servo positions
                outtakeLeftClosed = true;
            } else {
                robot.outtakeLeft.setPosition(0); //update new servo positions
                outtakeLeftClosed = false;
            }
        }

        //drone release
        if(impGamepad1.a.isInitialPress() || impGamepad1.b.isInitialPress() || impGamepad1.x.isInitialPress() || impGamepad1.y.isInitialPress()){
            robot.planeServo.setPosition(1); //need to update servo position
        }

        if(armAngle < 65){
            //servo angle flat to ground
            //when arm angle increases, servo angle decreases
        } else if (armAngle < 90){
            //servo angle = arm angle + 25
        } else if(armAngle < 190){
            //bottom of claw points same direction as arm
        } else if(armAngle < 270){
            //flat with backboard (60 degrees)
        }

        //slow mode for precision
//        if(impGamepad1.dpad_right.isInitialPress()){
//            if(slowMode == false) {
//                speedMod = 0.5;
//                slowMode = true;
//            } else if(slowMode == true){
//                speedMod = 0.9;
//                slowMode = false;
//            }
//        }

        telemetry.addData("arm position", robot.arm.getCurrentPosition());
        telemetry.addData("right trigger", impGamepad1.right_trigger.getValue());
        telemetry.addData("left trigger", impGamepad1.left_trigger.getValue());
        telemetry.update();
    }

    public double armPower(int target){

        error = target - robot.arm.getCurrentPosition();
        dArm = (error - pArm) / PIDTimer.seconds();
        iArm = iArm + (error * PIDTimer.seconds());
        pArm = error;
        armAngle = recalculateAngle();
        cosArm = Math.cos(Math.toRadians(armAngle));
        total = ((pArm * kp) + (iArm * ki) + (dArm * kd))/100 + (cosArm * kCos);
        PIDTimer.reset();

        if(iArm > iArmMax){
            iArm = iArmMax;
        }else if(iArm < -iArmMax){
            iArm = -iArmMax;
        }

        if(total > .5){
            total = .5;
        }

        return total;
    }

    public double recalculateAngle(){
        double calculatedAngle = 0;
        //find calculated Angle using ticks on motor
        return calculatedAngle;
    }
}