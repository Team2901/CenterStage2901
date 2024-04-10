package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.CombinedHardware;
import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;

import java.util.Locale;

@TeleOp (name = "Servo Tester", group = "Utilities")
public class ServoTester extends OpMode {

    CombinedHardware robot = new CombinedHardware();
    ImprovedGamepad impGamepad1;
    public double currentPosLeft = CombinedHardware.outtakeLeftOpenPos;
    public double currentPosRight = CombinedHardware.outtakeRightOpenPos;
    public double currentPosRotation = 0.5;
    public double currentPosPlane = 0.5;

    @Override
    public void init() {
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //impGamepad1 = new ImprovedGamepad(gamepad1, new ElapsedTime(), "impGamepad11", telemetry);
        impGamepad1 = new ImprovedGamepad(gamepad1, new ElapsedTime(), "gamepad1");
        robot.init(this.hardwareMap, telemetry);

        telemetry.addData("Init: outtakeLeft Position", robot.outtakeLeft.getPosition());
        telemetry.addData("Init: outtakeRight Position", robot.outtakeRight.getPosition());
        telemetry.addData("Init: rotationServo Position", robot.rotationServo.getPosition());
        telemetry.addData("Init: planeServo Position", robot.planeServo.getPosition());

        robot.outtakeLeft.setPosition(currentPosLeft);
        robot.outtakeRight.setPosition(currentPosRight);
        robot.rotationServo.setPosition(currentPosRotation);
        robot.planeServo.setPosition(currentPosPlane);
    }

    @Override
    public void loop() {
        impGamepad1.update();

        telemetry.addData("Help: D-pad Up", "outtakeLeft increase");
        telemetry.addData("Help: D-pad Down", "outtakeLeft decrease");
        telemetry.addData("Help: D-pad Left", "outtakeRight decrease");
        telemetry.addData("Help: D-pad Right", "outtakeRight increase");
        telemetry.addData("Help: Left Bumper", "rotationServo decrease");
        telemetry.addData("Help: Right Bumper", "rotationServo increase");
        telemetry.addData("Help: X", "planeServo decrease");
        telemetry.addData("Help: B", "planeServo increase");

        if(impGamepad1.dpad_up.isInitialPress()){
            currentPosLeft += 0.025;
            if (currentPosLeft > 1) currentPosLeft = 1.0;
            robot.outtakeLeft.setPosition(currentPosLeft);
        } else if(impGamepad1.dpad_down.isInitialPress()){
            currentPosLeft -= 0.025;
            if (currentPosLeft < 0) currentPosLeft = 0.0;
            robot.outtakeLeft.setPosition(currentPosLeft);
        }

        if(impGamepad1.dpad_left.isInitialPress()){
            currentPosRight -= 0.025;
            if (currentPosRight < 0) currentPosRight = 0.0;
            robot.outtakeRight.setPosition(currentPosRight);
        } else if(impGamepad1.dpad_right.isInitialPress()){
            currentPosRight += 0.025;
            if (currentPosRight > 1) currentPosRight = 1.0;
            robot.outtakeRight.setPosition(currentPosRight);
        }

        if(impGamepad1.left_bumper.isInitialPress()){
            currentPosRotation -= 0.025;
            if (currentPosRotation < 0) currentPosRotation = 0.0;
            robot.rotationServo.setPosition(currentPosRotation);
        } else if(impGamepad1.right_bumper.isInitialPress()){
            currentPosRotation += 0.025;
            if (currentPosRotation > 1) currentPosRotation = 1.0;
            robot.rotationServo.setPosition(currentPosRotation);
        }

        if(impGamepad1.x.isInitialPress()){
            currentPosPlane -= 0.025;
            if (currentPosPlane < 0) currentPosPlane = 0.0;
            robot.planeServo.setPosition(currentPosPlane);
        } else if(impGamepad1.b.isInitialPress()){
            currentPosPlane += 0.025;
            if (currentPosPlane > 1) currentPosPlane = 1.0;
            robot.planeServo.setPosition(currentPosPlane);
        }

        telemetry.addData("Servo: outtakeLeft Position", String.format(Locale.ENGLISH, "%.3f", robot.outtakeLeft.getPosition()));
        telemetry.addData("Servo: outtakeRight Position", String.format(Locale.ENGLISH, "%.3f", robot.outtakeRight.getPosition()));
        telemetry.addData("Servo: rotationServo Position", String.format(Locale.ENGLISH, "%.3f", robot.rotationServo.getPosition()));
        telemetry.addData("Servo: planeServo Position", String.format(Locale.ENGLISH, "%.3f", robot.planeServo.getPosition()));
    }
}
