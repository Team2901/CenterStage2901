package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.StatesHardware;
import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;

@TeleOp (name = "Servo Tester", group = "Utilities")
public class ServoTester extends OpMode {

    StatesHardware robot = new StatesHardware();
    ImprovedGamepad impGamepad1;
    public double currentPos = 0.5;
    public double currentPos2 = 0.5;

    @Override
    public void init() {
        impGamepad1 = new ImprovedGamepad(gamepad1, new ElapsedTime(), "impGamepad11", telemetry);
        robot.init(this.hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        impGamepad1.update();

        telemetry.addLine("D-pad Up and Down to change servo position");

        if(impGamepad1.dpad_up.isInitialPress()){
            robot.outtakeRight.setPosition(currentPos + 0.05);
            currentPos = robot.outtakeRight.getPosition();
        } else if(impGamepad1.dpad_down.isInitialPress()){
            robot.outtakeRight.setPosition(currentPos - 0.05);
            currentPos = robot.outtakeRight.getPosition();
        }

        if(impGamepad1.dpad_left.isInitialPress()){
            robot.outtakeLeft.setPosition(currentPos2 + 0.05);
            currentPos2 = robot.outtakeLeft.getPosition();
        } else if(impGamepad1.dpad_right.isInitialPress()){
            robot.outtakeLeft.setPosition(currentPos2 - 0.05);
            currentPos2 = robot.outtakeLeft.getPosition();
        }

        telemetry.addData("OuttakeRight Target", robot.outtakeRight.getPosition());
        telemetry.addData("OuttakeLeft Target", robot.outtakeLeft.getPosition());
    }
}
