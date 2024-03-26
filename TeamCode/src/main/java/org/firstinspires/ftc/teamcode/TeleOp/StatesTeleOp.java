package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.StatesHardware;
import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;

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
    public double armMod = 0.6;

    public int maxHeightArmTicks = 3390; //preset deliver point
    public int maxArmTicks = 5500;
    public int minArmTicks = 15;
    public int currentArmTicks = 0;

    public double startFrontLeft;

    public boolean slowMode = false;

    public boolean outtakeRightClosed = false;
    public boolean outtakeLeftClosed = false;
    public boolean armModFast = false;

    public double rotationServoPosition = 0.1;
    public double rotationServoMin = 0.1;
    public double rotationServoMax = 0.8;

    public static double outtakeLeftClosedPos = 0.315;
    public static double outtakeLeftOpenPos = outtakeLeftClosedPos - 0.2;
    public static double outtakeRightClosedPos = 0.3;
    public static double outtakeRightOpenPos = outtakeRightClosedPos - 0.2;


    double initArmAngle = 60.0;
    double armAngle = initArmAngle;

    double error = 0.0;
    double total = 0.0;
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double kCos = 0.3;
    double pArm = 0.0;
    double iArm = 0.0;
    double dArm = 0.0;
    double cosArm = 0.0;
    double iArmMax = 0.25;

    boolean fieldOriented = true;

    enum Alliance{
        RED,
        BLUE
    }
    Alliance alliance = Alliance.RED;
    @Override
    public void init() {
        impGamepad1 = new ImprovedGamepad(gamepad1, new ElapsedTime(), "gamepad1");
        impGamepad2 = new ImprovedGamepad(gamepad2, new ElapsedTime(), "gamepad2");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(this.hardwareMap, telemetry);

        startFrontLeft = robot.frontLeft.getCurrentPosition();

        outtakeTimer.startTime();
    }

    @Override
    public void init_loop() {
        impGamepad1.update();
        super.init_loop();
        telemetry.addLine("Red (B) or Blue (X) Alliance?");

        if(impGamepad1.x.isInitialPress()){
            alliance = Alliance.BLUE;
        }
        if(impGamepad1.b.isInitialPress()){
            alliance = Alliance.RED;
        }
        telemetry.addData("Alliance Color:", alliance);
        telemetry.update();

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

        if(Math.abs(rightStickXVal) > 0){
            rotate = rightStickXVal * turnMod;
        } else if(impGamepad1.x.isPressed()){
            rotate = -turnToAngle(alliance == Alliance.RED? 90: -90) * turnMod;
        } else if(impGamepad1.a.isPressed()){
            rotate = -turnToAngle(alliance == Alliance.RED? 45: -45)*turnMod;
        }
        else {
            rotate = 0;
        }

        if(impGamepad2.x.isInitialPress()){
            robot.imu.resetYaw();
        }
        double controllerAngle = AngleUnit.RADIANS.fromDegrees(impGamepad1.left_stick_angle);
        double robotAngle = AngleUnit.RADIANS.fromDegrees(robot.getAngle());
        double forward;
        double strafe;
        if(fieldOriented == true) {
            forward = impGamepad1.left_stick_radius * Math.cos(controllerAngle - robotAngle);
            strafe = impGamepad1.left_stick_radius * -Math.sin(controllerAngle - robotAngle);
        }
        else{
            forward = impGamepad1.left_stick_radius * Math.cos(controllerAngle);
            strafe = impGamepad1.left_stick_radius * -Math.sin(controllerAngle);
        }
        robot.backLeft.setPower(forward - strafe + rotate);
        robot.frontLeft.setPower(forward + strafe + rotate);
        robot.backRight.setPower(forward + strafe - rotate);
        robot.frontRight.setPower(forward - strafe - rotate);

        if(impGamepad1.back.isInitialPress()){
            fieldOriented = !fieldOriented;
        }
        if(Math.abs(robot.frontLeft.getPower()) > 0 || Math.abs(robot.frontRight.getPower()) > 0 || Math.abs(robot.backLeft.getPower()) > 0 || Math.abs(robot.backRight.getPower()) > 0){
            if(robot.arm.getCurrentPosition() < minArmTicks + 80) {
                currentArmTicks = minArmTicks + 80;
            }
        }

        //set arm to max height
        if(impGamepad1.dpad_up.isInitialPress()){
            currentArmTicks = maxHeightArmTicks;
        }

        //set arm to min height/ground
        if(impGamepad1.dpad_down.isInitialPress()){
            currentArmTicks = minArmTicks;
        }

        //fixing claw (outtake) positions with gamepad 1
        if(impGamepad1.dpad_left.isInitialPress()){
            robot.outtakeLeft.setPosition(robot.outtakeLeft.getPosition() + 0.015);
            outtakeLeftClosedPos = robot.outtakeLeft.getPosition();
        }
        if(impGamepad1.dpad_right.isInitialPress()){
            robot.outtakeRight.setPosition(robot.outtakeRight.getPosition() + 0.015);
            outtakeRightClosedPos = robot.outtakeRight.getPosition();
        }

        //arm up

        // commented out b/c of stability testing 3/7/2024 - should be added back in
        if(impGamepad1.right_trigger.getValue() > 0 && currentArmTicks < maxArmTicks){
            robot.arm.setPower(impGamepad1.right_trigger.getValue() * armMod);
            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            currentArmTicks = robot.arm.getCurrentPosition();
        } else if(impGamepad1.left_trigger.getValue() > 0 && currentArmTicks > minArmTicks){
            robot.arm.setPower(-impGamepad1.left_trigger.getValue() * armMod);
            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            currentArmTicks = robot.arm.getCurrentPosition();
        } else {
            robot.arm.setTargetPosition(currentArmTicks);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm.setPower(0.7 * armMod);
            //robot.arm.setPower(-0.9 * armMod);
        }

        //right claw toggle
        if(impGamepad1.right_bumper.isInitialPress()){
            if(!outtakeRightClosed) {
                robot.outtakeRight.setPosition(outtakeRightClosedPos);
                outtakeRightClosed = true;
            } else {
                //robot.outtakeRight.setPosition(outtakeRightOpenPos);
                robot.outtakeRight.setPosition(outtakeRightClosedPos-.203);
                outtakeRightClosed = false;
            }
        }

        //left claw toggle
        if(impGamepad1.left_bumper.isInitialPress()){
            if(!outtakeLeftClosed) {
                robot.outtakeLeft.setPosition(outtakeLeftClosedPos); //update new servo positions
                outtakeLeftClosed = true;
            } else {
                //robot.outtakeLeft.setPosition(outtakeLeftOpenPos); //update new servo positions
                robot.outtakeLeft.setPosition(outtakeLeftClosedPos-.2);
                outtakeLeftClosed = false;
            }
        }

        if(impGamepad1.b.isInitialPress()){
            if(!armModFast){
                armMod = 1;
                armModFast = true;
            } else {
                armMod = 0.6;
                armModFast = false;
            }
        }

        if(armAngle < 75){
            robot.rotationServo.setPosition(0.125);
            //when arm angle increases, servo angle decreases
        } else if (armAngle < 93){ //90 degrees, but a little bit of error in the math so have to adjust manually
            robot.rotationServo.setPosition(0.1);
            //servo angle = arm angle + 25
            //ethan says "zone between 65 and 90 that keeps the servo at the arm angle+25 degrees (so it is as close to horizontal as possible)"
        } else if(armAngle < 190){
            robot.rotationServo.setPosition(0.325);
        } else if(armAngle < 270){
            robot.rotationServo.setPosition(0.75 - ((armAngle - 190) * 0.004));
        }

        if(armAngle > 93 && armAngle < 190){
            armMod = 1;
        } else if (!armModFast){
            armMod = 0.6;
        }

        // GAMEPAD 2 CONTROLS

        //drone release
        if(impGamepad2.right_trigger.getValue() > 0 || impGamepad2.left_trigger.getValue() > 0){
            robot.planeServo.setPosition(1); //need to update servo position
        }

        //adjust outtakeRight closed position in case it skips (gamepad2)
        if(impGamepad2.y.isInitialPress()){
            robot.outtakeRight.setPosition(robot.outtakeRight.getPosition() + 0.015);
            outtakeRightClosedPos = robot.outtakeRight.getPosition();
        } else if(impGamepad2.a.isInitialPress()){
            robot.outtakeRight.setPosition(robot.outtakeRight.getPosition() - 0.015);
            outtakeRightClosedPos = robot.outtakeRight.getPosition();
        }

        //adjust outtakeLeft closed position in case it skips (gamepad2)
        if(impGamepad2.dpad_down.isInitialPress()){
            robot.outtakeLeft.setPosition(robot.outtakeLeft.getPosition() - 0.015);
            outtakeLeftClosedPos = robot.outtakeLeft.getPosition();
        } else if(impGamepad2.dpad_up.isInitialPress()){
            robot.outtakeLeft.setPosition(robot.outtakeLeft.getPosition() + 0.015);
            outtakeLeftClosedPos = robot.outtakeLeft.getPosition();
        }

        //min at 0.1 and max at 0.8 (wrist)

        //adjust rotationServo/wrist position manually
        if(impGamepad2.right_bumper.isInitialPress()){
            robot.rotationServo.setPosition(rotationServoPosition + 0.1);
            rotationServoPosition = robot.rotationServo.getPosition();
        } else if(impGamepad2.left_bumper.isInitialPress()){
            robot.rotationServo.setPosition(rotationServoPosition - 0.1);
            rotationServoPosition = robot.rotationServo.getPosition();
        }

        //gets rid of arm min ticks
        if(impGamepad2.b.isInitialPress()){
            minArmTicks = -10000;
        }

        //slow mode for precision - if needed, have to use a different button (dpad right is used for outtakeRight fixes)
//        if(impGamepad1.dpad_right.isInitialPress()){
//            if(slowMode == false) {
//                speedMod = 0.5;
//                slowMode = true;
//            } else if(slowMode == true){
//                speedMod = 0.9;
//                slowMode = false;
//            }
//        }

        armAngle = recalculateAngle();

        telemetry.addData("arm position", robot.arm.getCurrentPosition());
        telemetry.addData("arm target position", currentArmTicks);
        telemetry.addData("arm.getTargetPosition", robot.arm.getTargetPosition());
        telemetry.addData("arm power", robot.arm.getPower());
        telemetry.addData("arm angle", armAngle);
        telemetry.addData("right trigger", impGamepad1.right_trigger.getValue());
        telemetry.addData("left trigger", impGamepad1.left_trigger.getValue());
        telemetry.addData("outtakeLeft servo position", robot.outtakeLeft.getPosition());
        telemetry.addData("outtakeRight servo position", robot.outtakeRight.getPosition());
        telemetry.addData("rotation servo position", robot.rotationServo.getPosition());
        telemetry.addData("angle", robot.getAngle());
        telemetry.addData("joystick radius", impGamepad1.left_stick_radius);
        telemetry.addData("joystick angle", impGamepad1.left_stick_angle);
        telemetry.addData("left stick x value", impGamepad1.left_stick.x.getValue());
        telemetry.addData("left stick y value", impGamepad1.left_stick_y.getValue());
        telemetry.addData("Field Oriented:", fieldOriented);
        telemetry.addData("Alliance Color:", alliance);
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

    //fixed tick count for new 60rpm motors
    public double recalculateAngle(){
        double calculatedAngle;
        calculatedAngle = ((210*robot.arm.getCurrentPosition())/5360) + initArmAngle;
        return calculatedAngle;
    }

    public double turnToAngle(double turnAngle){

        //robot.getAngle is between -180 and 180, starting at 0
        double turnPower = 0;
        double targetAngle = AngleUnit.normalizeDegrees(turnAngle);
        double startAngle = robot.getAngle();
        double turnError = AngleUnit.normalizeDegrees(targetAngle - startAngle);
        if(!(turnError < .5 && turnError > -.5)){
            if(turnError >= 0){
                turnPower = turnError/50;
                if(turnPower > .75){
                    turnPower = .75;
                }
            }else if(turnError < 0){
                turnPower = turnError/50;
                if(turnPower < -.75){
                    turnPower = -.75;
                }
            }
//            robot.frontLeft.setPower(-turnPower);
//            robot.frontRight.setPower(turnPower);
//            robot.backLeft.setPower(-turnPower);
//            robot.backRight.setPower(turnPower);

            double currentAngle = robot.getAngle();
            turnError = AngleUnit.normalizeDegrees(targetAngle - currentAngle);
        }
//        robot.frontLeft.setPower(0);
//        robot.frontRight.setPower(0);
//        robot.backRight.setPower(0);
//        robot.backLeft.setPower(0);

        return turnPower;
    }
}