package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.StatesHardware;
import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;

@TeleOp(name = "Motor Tester", group = "Utilities")
public class MotorTester extends OpMode {

    StatesHardware robot = new StatesHardware();
    ImprovedGamepad impGamepad1;

    @Override
    public void init() {
        impGamepad1 = new ImprovedGamepad(gamepad1, new ElapsedTime(), "impGamepad11", telemetry);
        robot.init(this.hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        telemetry.addLine("Test Individual Motors");
        telemetry.addLine("Press A to turn FR");
        telemetry.addLine("Press Y to turn FL");
        telemetry.addLine("Press X to turn BR");
        telemetry.addLine("Press B to turn BL");

        impGamepad1.update();
        if (impGamepad1.a.isPressed()) {
            robot.frontRight.setPower(0.5);
        } else if (impGamepad1.dpad_down.isPressed()) {
            robot.frontRight.setPower(-0.5);
        } else {
            robot.frontRight.setPower(0);
        }
        if (impGamepad1.x.isPressed()) {
            robot.backRight.setPower(0.5);
        } else if (impGamepad1.dpad_right.isPressed()) {
            robot.backRight.setPower(-0.5);
        } else {
            robot.backRight.setPower(0);
        }
        if (impGamepad1.b.isPressed()) {
            robot.backLeft.setPower(0.5);
        } else if (impGamepad1.dpad_left.isPressed()) {
            robot.backLeft.setPower(-0.5);
        } else {
            robot.backLeft.setPower(0);
        }
        if (impGamepad1.y.isPressed()) {
            robot.frontLeft.setPower(0.5);
        } else if (impGamepad1.dpad_up.isPressed()) {
            robot.frontLeft.setPower(-0.5);
        } else {
            robot.frontLeft.setPower(0);
        }
        if (impGamepad1.right_bumper.isPressed()) {
            robot.arm.setPower(0.5);
        } else if (impGamepad1.left_bumper.isPressed()) {
            robot.arm.setPower(-0.5);
        } else {
            robot.arm.setPower(0);
        }
    }
}
