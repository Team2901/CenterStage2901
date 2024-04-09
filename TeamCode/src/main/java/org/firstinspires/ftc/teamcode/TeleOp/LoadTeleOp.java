package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.StatesHardware;
import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;

@TeleOp(name = "LoadTeleOp", group = "TeleOp")

public class LoadTeleOp extends OpMode {
    public double startFrontLeft;

    public boolean outtakeRightClosed = false;
    public boolean outtakeLeftClosed = false;

    public double outtakeLeftClosedPos = StatesHardware.outtakeLeftClosedPos;
    public double outtakeLeftOpenPos = StatesHardware.outtakeLeftOpenPos;
    public double outtakeRightClosedPos = StatesHardware.outtakeRightClosedPos;
    public double outtakeRightOpenPos = StatesHardware.outtakeRightOpenPos;

    StatesHardware robot = new StatesHardware();
    ImprovedGamepad impGamepad1;
    @Override
    public void init() {
        impGamepad1 = new ImprovedGamepad(gamepad1, new ElapsedTime(), "gamepad1");

        robot.init(this.hardwareMap, telemetry);

        //startFrontLeft = robot.frontLeft.getCurrentPosition();
    }

    @Override
    public void loop() {
        impGamepad1.update();

        if(impGamepad1.right_bumper.isInitialPress()){
            if(!outtakeRightClosed) {
                robot.outtakeRight.setPosition(outtakeRightClosedPos); //update new servo positions
                outtakeRightClosed = true;
            } else {
                robot.outtakeRight.setPosition(outtakeRightOpenPos); //update new servo positions
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

        if(impGamepad1.y.isInitialPress()){
            robot.outtakeRight.setPosition(outtakeRightClosedPos + 0.025);
            outtakeRightClosedPos = robot.outtakeRight.getPosition();
        } else if(impGamepad1.a.isInitialPress()){
            robot.outtakeRight.setPosition(outtakeRightClosedPos - 0.025);
            outtakeRightClosedPos = robot.outtakeRight.getPosition();
        }

        //adjust outtakeLeft closed position in case it skips (gamepad2)
        if(impGamepad1.dpad_down.isInitialPress()){
            robot.outtakeLeft.setPosition(outtakeLeftClosedPos + 0.025);
            outtakeLeftClosedPos = robot.outtakeLeft.getPosition();
        } else if(impGamepad1.dpad_up.isInitialPress()){
            robot.outtakeLeft.setPosition(outtakeLeftClosedPos - 0.025);
            outtakeLeftClosedPos = robot.outtakeLeft.getPosition();
        }

        if(impGamepad1.b.isInitialPress()){
            robot.rotationServo.setPosition(StatesHardware.rotationServoInitPos);
        }
        if(impGamepad1.x.isInitialPress()){
            robot.rotationServo.setPosition(StatesHardware.rotationServoMin);
        }

        telemetry.addData("outtake left pos", robot.outtakeLeft.getPosition());
        telemetry.addData("outtake right pos", robot.outtakeRight.getPosition());
        telemetry.update();
    }
}
