package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.CombinedHardware;
import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;

@TeleOp(name = "Distance Sensor Testing", group = "TeleOp")
public class DistanceSensorTesting extends OpMode {

    CombinedHardware robot = new CombinedHardware();
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

    public int maxHeightArmTicks = 1700; //max height (straight vert)
    public int maxArmTicks = 2670;
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

//    public static double outtakeLeftClosedPos = 0.335;
//    public static double outtakeLeftOpenPos = 0.15;
//    public static double outtakeRightClosedPos = 0.483;
//    public static double outtakeRightOpenPos = 0.280;

    //coachbot settings
    public static double outtakeLeftClosedPos = 0.255;
    public static double outtakeLeftOpenPos = 0.105;
    public static double outtakeRightClosedPos = 0.255;
    public static double outtakeRightOpenPos = 0.105;

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
        leftStickXVal = -impGamepad1.left_stick.x.getValue() * speedMod;
        leftStickYVal = impGamepad1.left_stick.y.getValue() * speedMod;
        rightStickXVal = impGamepad1.right_stick.x.getValue() * speedMod;
        rightStickYVal = impGamepad1.right_stick.y.getValue() * speedMod;

        rotate = rightStickXVal * turnMod;

        robot.backLeft.setPower(leftStickYVal+leftStickXVal+rotate);
        robot.frontLeft.setPower(leftStickYVal-leftStickXVal+rotate);
        robot.backRight.setPower(leftStickYVal-leftStickXVal-rotate);
        robot.frontRight.setPower(leftStickYVal+leftStickXVal-rotate);

        telemetry.addData("The ROTATION:", startFrontLeft - robot.frontLeft.getCurrentPosition());

        //might have to use PID for the arm, we'll see

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
        }

        //right claw toggle
        if(impGamepad1.right_bumper.isInitialPress()){
            if(!outtakeRightClosed) {
                robot.outtakeRight.setPosition(outtakeRightClosedPos);
                outtakeRightClosed = true;
            } else {
                robot.outtakeRight.setPosition(outtakeRightOpenPos);
                outtakeRightClosed = false;
            }
        }

        //left claw toggle
        if(impGamepad1.left_bumper.isInitialPress()){
            if(!outtakeLeftClosed) {
                robot.outtakeLeft.setPosition(outtakeLeftClosedPos); //update new servo positions
                outtakeLeftClosed = true;
            } else {
                robot.outtakeLeft.setPosition(outtakeLeftOpenPos); //update new servo positions
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

//        if(impGamepad1.a.isInitialPress()){
//            double distanceFromBoard = robot.distanceSensorLeft.getDistance(DistanceUnit.INCH);
//            if(distanceFromBoard > 2){
//                moveInches(distanceFromBoard - 2);
//            }
//            robot.outtakeRight.setPosition(outtakeRightOpenPos);
//            robot.outtakeLeft.setPosition(outtakeLeftOpenPos);
//        }

        if(impGamepad1.a.isPressed()){
            double distanceFromBoard = robot.distanceSensorLeft.getDistance(DistanceUnit.INCH);
            while(distanceFromBoard > 2){
                drivePower(0.35);
            }
            drivePower(0);
            robot.outtakeRight.setPosition(outtakeRightOpenPos);
            robot.outtakeLeft.setPosition(outtakeLeftOpenPos);
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
            robot.outtakeLeft.setPosition(robot.outtakeLeft.getPosition() + 0.015);
            outtakeLeftClosedPos = robot.outtakeLeft.getPosition();
        } else if(impGamepad2.dpad_up.isInitialPress()){
            robot.outtakeLeft.setPosition(robot.outtakeLeft.getPosition() - 0.015);
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

        telemetry.addData("Arm Position", robot.arm.getCurrentPosition());
        telemetry.addData("Arm Angle", armAngle);
        telemetry.addData("Right Trigger", impGamepad1.right_trigger.getValue());
        telemetry.addData("Left Trigger", impGamepad1.left_trigger.getValue());
        telemetry.addData("OuttakeLeft Servo Position", robot.outtakeLeft.getPosition());
        telemetry.addData("OuttakeRight Servo Position", robot.outtakeRight.getPosition());
        telemetry.addData("Rotation Servo Position", robot.rotationServo.getPosition());
        telemetry.addLine();
        telemetry.addData("Left Distance (in)", robot.distanceSensorLeft.getDistance(DistanceUnit.INCH));
        telemetry.addData("Right Distance (in)", robot.distanceSensorRight.getDistance(DistanceUnit.INCH));
        telemetry.addData("Left Color RED", robot.colorSensorLeft.red());
        telemetry.addData("Left Color GREEN", robot.colorSensorLeft.green());
        telemetry.addData("Left Color BLUE", robot.colorSensorLeft.blue());
        telemetry.addData("Right Color RED", robot.colorSensorRight.red());
        telemetry.addData("Right Color GREEN", robot.colorSensorRight.green());
        telemetry.addData("Right Color BLUE", robot.colorSensorRight.blue());
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
        double calculatedAngle;
        calculatedAngle = ((210*robot.arm.getCurrentPosition())/2440) + initArmAngle;
        return calculatedAngle;
    }

    private void moveInches(double inches) {
        int ticks = (int) (inches * robot.TICKS_PER_INCH);

        robot.frontLeft.setTargetPosition(robot.frontLeft.getCurrentPosition() + ticks);
        robot.frontRight.setTargetPosition(robot.frontRight.getCurrentPosition() + ticks);
        robot.backLeft.setTargetPosition(robot.backLeft.getCurrentPosition() + ticks);
        robot.backRight.setTargetPosition(robot.backRight.getCurrentPosition() + ticks);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drivePower(0.35);

        while (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy()) {
//            telemetry.addData("Current Left Position", robot.backLeft.getCurrentPosition());
//            telemetry.addData("Current Right
//            public void onError ( int errorCode){
//
//            }
        }
    }

    private void drivePower(double power){
        robot.frontLeft.setPower(power);
        robot.frontRight.setPower(power);
        robot.backLeft.setPower(power);
        robot.backRight.setPower(power);
    }
}