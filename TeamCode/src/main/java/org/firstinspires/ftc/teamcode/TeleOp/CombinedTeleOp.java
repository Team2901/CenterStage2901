package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Color;

import androidx.annotation.ColorInt;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.CombinedHardware;
import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.util.Blinkin;
import org.opencv.core.Scalar;

@TeleOp(name = "Worlds Mecanum TeleOp", group = "1TeleOp")
public class CombinedTeleOp extends OpMode {

    CombinedHardware robot = new CombinedHardware();
    ImprovedGamepad impGamepad1;
    ImprovedGamepad impGamepad2;
    ElapsedTime outtakeTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    Blinkin lights;

    long startTime;

    public static final boolean USE_LIGHTS = true;

    public double rotate;
//    public double speedMod = 0.9;
    public double turnMod = 0.9;
    public double armMod = 0.6;

    public int currentArmTicks = 0;

    public boolean slowMode = false;

    public boolean outtakeRightClosed = false;
    public boolean outtakeLeftClosed = false;
    public boolean armModFast = false;

    public double rotationServoPosition = 0.1;

    boolean fieldOriented = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        impGamepad1 = new ImprovedGamepad(gamepad1, new ElapsedTime(), "gamepad1", telemetry);
        impGamepad2 = new ImprovedGamepad(gamepad2, new ElapsedTime(), "gamepad2", telemetry);
        robot.init(this.hardwareMap, telemetry);
        if (USE_LIGHTS) {
            lights = new Blinkin(hardwareMap.get(RevBlinkinLedDriver.class, "blinkinL"), hardwareMap.get(RevBlinkinLedDriver.class, "blinkinR"));
//            lights.setPixelStatus(Blinkin.PixelStatus.TELEOP);
            lights.setPixelLeftStatus(Blinkin.PixelStatus.TELEOP);
            lights.setPixelRightStatus(Blinkin.PixelStatus.TELEOP);
        }
        currentArmTicks = robot.arm.getCurrentPosition();

        startTime = System.currentTimeMillis();
        outtakeTimer.startTime();
    }

    @Override
    public void init_loop() {
        impGamepad1.update();

        telemetry.addData("Help", "Red (B) or Blue (X) Alliance?");
        if (impGamepad1.x.isInitialPress()) {
            robot.alliance = CombinedHardware.Alliance.BLUE;
        }
        if (impGamepad1.b.isInitialPress()) {
            robot.alliance = CombinedHardware.Alliance.RED;
        }
        telemetry.addData("Current Alliance:", robot.alliance);
    }

    @Override
    public void loop() {
        impGamepad1.update();
        impGamepad2.update();

        double armAngle = robot.recalculateAngle();

        //basic drive base stuff
        if (Math.abs(impGamepad1.right_stick.x.getValue()) > 0) {
            rotate = impGamepad1.right_stick.x.getValue() * turnMod;
        } else if (impGamepad1.x.isPressed()) {
            rotate = -robot.turnToAngle(robot.alliance == CombinedHardware.Alliance.RED ? 90 : -90) * turnMod;
        } else if (impGamepad1.a.isPressed() && !impGamepad1.start.isPressed()) {
            rotate = -robot.turnToAngle(robot.alliance == CombinedHardware.Alliance.RED ? 45 : -45) * turnMod;
        } else {
            rotate = 0;
        }

        if (impGamepad2.x.isInitialPress()) {
            robot.imu.resetYaw();
        }

        double controllerAngle = AngleUnit.RADIANS.fromDegrees(impGamepad1.left_stick.angle.getValue());
        double robotAngle = AngleUnit.RADIANS.fromDegrees(robot.getAngle());
        double forward;
        double strafe;

        if (fieldOriented) {
            if (impGamepad1.y.isPressed()) {
                forward = -robot.approachBackdrop();
                strafe = 0;
                if(USE_LIGHTS && robot.distanceSensorLeft.getDistance(DistanceUnit.INCH) < 1.9) lights.setPixelStatus(Blinkin.PixelStatus.ALIGNED);

            } else {
                forward = impGamepad1.left_stick.radius.getValue() * Math.cos(controllerAngle - robotAngle);
                strafe = impGamepad1.left_stick.radius.getValue() * -Math.sin(controllerAngle - robotAngle);
            }
        } else {
            if (impGamepad1.y.isPressed()) {
                forward = -robot.approachBackdrop();
                strafe = 0;
                if(USE_LIGHTS && robot.distanceSensorLeft.getDistance(DistanceUnit.INCH) < 1.9) lights.setPixelStatus(Blinkin.PixelStatus.ALIGNED);
                telemetry.addData("forward power", forward);
            } else {
                forward = impGamepad1.left_stick.radius.getValue() * Math.cos(controllerAngle);
                strafe = impGamepad1.left_stick.radius.getValue() * -Math.sin(controllerAngle);
            }
        }

        robot.backLeft.setPower(forward - strafe + rotate);
        robot.frontLeft.setPower(forward + strafe + rotate);
        robot.backRight.setPower(forward + strafe - rotate);
        robot.frontRight.setPower(forward - strafe - rotate);

        if (impGamepad2.dpad_left.isInitialPress()) {
            robot.alliance = CombinedHardware.Alliance.BLUE;
        }
        if (impGamepad2.dpad_right.isInitialPress()) {
            robot.alliance = CombinedHardware.Alliance.RED;
        }

        if (impGamepad1.back.isInitialPress()) {
            fieldOriented = !fieldOriented;
        }

        //commented out because you can't drive up to a pixel or push pixels around
//        if(Math.abs(robot.frontLeft.getPower()) > 0 || Math.abs(robot.frontRight.getPower()) > 0 || Math.abs(robot.backLeft.getPower()) > 0 || Math.abs(robot.backRight.getPower()) > 0){
//            if(robot.arm.getCurrentPosition() < CombinedHardware.minArmTicks + 80) {
//                currentArmTicks = robot.arm.getCurrentPosition() + 80;
//            }
//        }

        //set arm to max height
        if (impGamepad1.dpad_up.isInitialPress()) {
            currentArmTicks = CombinedHardware.maxHeightArmTicks;
        }

        //set arm to min height/ground
        if (impGamepad1.dpad_down.isInitialPress()) {
            currentArmTicks = CombinedHardware.minArmTicks;
        }

        //fixing claw (outtake) positions with gamepad 1
        if (impGamepad1.dpad_left.isInitialPress()) {
            robot.outtakeLeft.setPosition(robot.outtakeLeft.getPosition() + 0.015);
            robot.outtakeLeftClosedPos = robot.outtakeLeft.getPosition();
        }
        if (impGamepad1.dpad_right.isInitialPress()) {
            robot.outtakeRight.setPosition(robot.outtakeRight.getPosition() + 0.015);
            robot.outtakeRightClosedPos = robot.outtakeRight.getPosition();
        }

        //arm up
        if (impGamepad1.right_trigger.getValue() > 0 && currentArmTicks < CombinedHardware.maxArmTicks) {
            robot.arm.setPower(impGamepad1.right_trigger.getValue() * armMod);
            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            currentArmTicks = robot.arm.getCurrentPosition();
        } else if (impGamepad1.left_trigger.getValue() > 0 && currentArmTicks > CombinedHardware.minArmTicks) { // got rid of the minimum arm ticks limit
            robot.arm.setPower(-impGamepad1.left_trigger.getValue() * armMod);
            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            currentArmTicks = robot.arm.getCurrentPosition();
        } else {
            robot.arm.setTargetPosition(currentArmTicks);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm.setPower(0.7 * armMod);
            //robot.arm.setPower(-0.9 * armMod);
        }

//        if(gamepad1.right_trigger > 0 && currentArmTicks < CombinedHardware.maxArmTicks){
//            robot.arm.setPower(gamepad1.right_trigger * armMod);
//            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            currentArmTicks = robot.arm.getCurrentPosition();
//        } else if(gamepad1.left_trigger > 0 && currentArmTicks > CombinedHardware.minArmTicks){
//            robot.arm.setPower(-gamepad1.left_trigger * armMod);
//            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            currentArmTicks = robot.arm.getCurrentPosition();
//        } else {
//            robot.arm.setTargetPosition(currentArmTicks);
//            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.arm.setPower(0.7 * armMod);
//            //robot.arm.setPower(-0.9 * armMod);
//        }

        //right claw toggle
        if (impGamepad1.right_bumper.isInitialPress()) {
            if (!outtakeRightClosed) {
                robot.outtakeRight.setPosition(robot.outtakeRightClosedPos);
                outtakeRightClosed = true;
//                if (USE_LIGHTS) lights.setPixelRightStatus(Blinkin.PixelStatus.HELD);
            } else {
                //robot.outtakeRight.setPosition(outtakeRightOpenPos);
                robot.outtakeRight.setPosition(robot.outtakeRightOpenPos);
                outtakeRightClosed = false;
                if (USE_LIGHTS) lights.setPixelRightStatus(Blinkin.PixelStatus.TELEOP);
            }
        }

        //left claw toggle
        if (impGamepad1.left_bumper.isInitialPress()) {
            if (!outtakeLeftClosed) {
                robot.outtakeLeft.setPosition(robot.outtakeLeftClosedPos);
                outtakeLeftClosed = true;
//                if (USE_LIGHTS) lights.setPixelLeftStatus(Blinkin.PixelStatus.HELD);
            } else {
                robot.outtakeLeft.setPosition(robot.outtakeLeftOpenPos);
                outtakeLeftClosed = false;
                if (USE_LIGHTS) lights.setPixelLeftStatus(Blinkin.PixelStatus.TELEOP);
            }
        }

        if (impGamepad1.b.isInitialPress()) {
            if (!armModFast) {
                armMod = 1;
                armModFast = true;
            } else {
                armMod = 0.6;
                armModFast = false;
            }
        }

        if (armAngle > 93 && armAngle < 190) {
            armMod = 1;
        } else if (!armModFast) {
            armMod = 0.6;
        }

        robot.adjustWrist();

        // GAMEPAD 2 CONTROLS

        //drone release
        if (impGamepad2.right_trigger.getValue() > 0 || impGamepad2.left_trigger.getValue() > 0) {
            robot.planeServo.setPosition(CombinedHardware.planeServoReleasePos);
        }

        //adjust outtakeRight closed position in case it skips (gamepad2)
//        if (impGamepad2.y.isInitialPress()) {
//            robot.outtakeRight.setPosition(robot.outtakeRight.getPosition() + 0.015);
//            robot.outtakeRightClosedPos = robot.outtakeRight.getPosition();
//        } else if (impGamepad2.a.isInitialPress()) {
//            robot.outtakeRight.setPosition(robot.outtakeRight.getPosition() - 0.015);
//            robot.outtakeRightClosedPos = robot.outtakeRight.getPosition();
//        }
//
//        //adjust outtakeLeft closed position in case it skips (gamepad2)
//        if (impGamepad2.dpad_down.isInitialPress()) {
//            robot.outtakeLeft.setPosition(robot.outtakeLeft.getPosition() - 0.015);
//            robot.outtakeLeftClosedPos = robot.outtakeLeft.getPosition();
//        } else if (impGamepad2.dpad_up.isInitialPress()) {
//            robot.outtakeLeft.setPosition(robot.outtakeLeft.getPosition() + 0.015);
//            robot.outtakeLeftClosedPos = robot.outtakeLeft.getPosition();
//        }

        //min at 0.1 and max at 0.8 (wrist)

        //adjust rotationServo/wrist position manually
        if (impGamepad2.right_bumper.isInitialPress()) {
            robot.rotationServo.setPosition(rotationServoPosition + 0.1);
            rotationServoPosition = robot.rotationServo.getPosition();
        } else if (impGamepad2.left_bumper.isInitialPress()) {
            robot.rotationServo.setPosition(rotationServoPosition - 0.1);
            rotationServoPosition = robot.rotationServo.getPosition();
        }

        //gets rid of arm min ticks
        if (impGamepad2.b.isInitialPress()) {
            CombinedHardware.minArmTicks = -10000;
        }

        // Reset arm encoder
        // (We removed it from init, so now it is a button press in case it is needed)
        if (impGamepad2.back.isInitialPress()) {
            DcMotor.RunMode currentMode = robot.arm.getMode();
            robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.arm.setMode(currentMode);
        }

        if(impGamepad2.dpad_right.isInitialPress()){
            CombinedHardware.minArmTicks = robot.arm.getCurrentPosition();
        }

        if (USE_LIGHTS) {
            if (impGamepad2.left_stick.x.getValue() < 0) {
                lights.setPixelLeftStatus(Blinkin.PixelStatus.REQUESTED);
                if (impGamepad2.a.isInitialPress()) {
                    lights.setPixelLeft(Blinkin.PixelColor.GREEN);
                }
                if (impGamepad2.y.isInitialPress()) {
                    lights.setPixelLeft(Blinkin.PixelColor.YELLOW);
                }
                if (impGamepad2.dpad_down.isInitialPress()) {
                    lights.setPixelLeft(Blinkin.PixelColor.PURPLE);
                }
                if (impGamepad2.dpad_up.isInitialPress()) {
                    lights.setPixelLeft(Blinkin.PixelColor.WHITE);
                }
            }
            else if(impGamepad2.left_stick.x.getValue() > 0) {
                lights.setPixelRightStatus(Blinkin.PixelStatus.REQUESTED);
                if (impGamepad2.a.isInitialPress()) {
                    lights.setPixelRight(Blinkin.PixelColor.GREEN);
                }
                if (impGamepad2.y.isInitialPress()) {
                    lights.setPixelRight(Blinkin.PixelColor.YELLOW);
                }
                if (impGamepad2.dpad_down.isInitialPress()) {
                    lights.setPixelRight(Blinkin.PixelColor.PURPLE);
                }
                if (impGamepad2.dpad_up.isInitialPress()) {
                    lights.setPixelRight(Blinkin.PixelColor.WHITE);
                }
            }
            NormalizedRGBA leftColorN = robot.colorSensorLeft.getNormalizedColors();
            NormalizedRGBA rightColorN = robot.colorSensorLeft.getNormalizedColors();

            boolean leftPixelDetected = true;
            boolean rightPixelDetected = true;

            float[] hsv = new float[3];

            Color.RGBToHSV((int) (leftColorN.red*255), (int) (leftColorN.green*255), (int) (leftColorN.blue*255), hsv);
            telemetry.addData("hue", hsv[0]);
            telemetry.addData("sat",hsv[1]);
            telemetry.addData("val",hsv[2]);

//            Color.RGBToHSV((int) (rightColorN.red*255), (int) (rightColorN.green*255), (int) (rightColorN.blue*255), hsv);


//            boolean leftPixelDetected = false;
//            if(leftColor.red > 0.2 && leftColor.green > 0.2 && leftColor.blue > 0.2){
//                leftPixelDetected = true;
//            }
//
//            boolean rightPixelDetected = false;
//            if(rightColor.red > 0.2 && rightColor.green > 0.2 && rightColor.blue > 0.2){
//                rightPixelDetected = true;
//            }

//            telemetry.addData("left color sensor",leftColor);
//            telemetry.addData("right color sensor",rightColor);

            if(robot.colorSensorLeft.getDistance(DistanceUnit.INCH) < CombinedHardware.colorSensorPixelDistance && lights.getPixelLeftStatus() == Blinkin.PixelStatus.REQUESTED && leftPixelDetected){
                lights.setPixelLeftStatus(Blinkin.PixelStatus.HELD);
            }
            if(robot.colorSensorRight.getDistance(DistanceUnit.INCH) < CombinedHardware.colorSensorPixelDistance && lights.getPixelRightStatus() == Blinkin.PixelStatus.REQUESTED && rightPixelDetected){
                lights.setPixelRightStatus(Blinkin.PixelStatus.HELD);
            }

            int elapsedTime = (int) ((System.currentTimeMillis() - startTime)/ 1000);
            telemetry.addData("elapsedTime",elapsedTime);
            if(elapsedTime >= 87 && elapsedTime <= 90){
                lights.update(7);
            }else {
                lights.update();
            }
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

        telemetry.addData("Field Oriented:", fieldOriented);
        telemetry.addData("Alliance Color:", robot.alliance);
        telemetry.addLine();
        telemetry.addData("outtakeLeft servo position", robot.outtakeLeft.getPosition());
        telemetry.addData("outtakeRight servo position", robot.outtakeRight.getPosition());
        telemetry.addData("arm position", robot.arm.getCurrentPosition());
        telemetry.addData("arm target position", currentArmTicks);
        telemetry.addData("arm power", robot.arm.getPower());
        telemetry.addData("arm angle", armAngle);
        telemetry.addData("right trigger", impGamepad1.right_trigger.getValue());
        telemetry.addData("left trigger", impGamepad1.left_trigger.getValue());
        telemetry.addData("rotation servo position", robot.rotationServo.getPosition());
        telemetry.addData("angle", robot.getAngle());
        telemetry.addData("joystick radius", impGamepad1.left_stick.radius.getValue());
        telemetry.addData("joystick angle", impGamepad1.left_stick.angle.getValue());
        telemetry.addData("left stick x value", impGamepad1.left_stick.x.getValue());
        telemetry.addData("left stick y value", impGamepad1.left_stick.y.getValue());
        telemetry.addData("distance", robot.distanceSensorLeft.getDistance(DistanceUnit.INCH));
        telemetry.addData("forward", forward);
        telemetry.update();
    }
}