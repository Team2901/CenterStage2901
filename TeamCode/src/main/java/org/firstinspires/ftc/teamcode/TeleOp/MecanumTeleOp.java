package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.Hardware.MecanumDriveHardware;

@TeleOp(name = "Mecanum Base", group = "TeleOp")
public class MecanumTeleOp extends OpMode {



    MecanumDriveHardware robot = new MecanumDriveHardware();
    ImprovedGamepad impGamepad1;

    public double rightStickYVal;
    public double rightStickXVal;
    public double leftStickYVal;
    public double leftStickXVal;
    public double rotate;
    public double speedMod = 0.5;
    public double turnMod = 0.6;
    public double liftMod = 1;

    @Override
    public void init() {
        impGamepad1 = new ImprovedGamepad(gamepad1, new ElapsedTime(), "gamepad1");
        robot.init(this.hardwareMap);
    }

    @Override
    public void loop() {
        impGamepad1.update();

        leftStickXVal = impGamepad1.left_stick_x.getValue() * speedMod;
        leftStickYVal = impGamepad1.left_stick_y.getValue() * speedMod;
        rightStickXVal = impGamepad1.right_stick_x.getValue() * speedMod;
        rightStickYVal = impGamepad1.right_stick_y.getValue() * speedMod;

        rotate = rightStickXVal * turnMod;

        robot.backLeft.setPower(leftStickYVal-leftStickXVal+rotate);
        robot.frontLeft.setPower(leftStickYVal+leftStickXVal+rotate);
        robot.backRight.setPower(leftStickYVal+leftStickXVal-rotate);
        robot.frontRight.setPower(leftStickYVal-leftStickXVal-rotate);

        if(robot.lift.getCurrentPosition() > -7500 && impGamepad1.right_trigger.getValue() != 0) {
            if (impGamepad1.right_trigger.getValue() > 0) {
                robot.lift.setPower(-impGamepad1.right_trigger.getValue() * liftMod);
            } else {
                robot.lift.setPower(0);
            }
        } else if(robot.lift.getCurrentPosition() <= -100) {
            if (impGamepad1.left_trigger.getValue() > 0) {
                robot.lift.setPower(impGamepad1.left_trigger.getValue() * liftMod);
            } else {
                robot.lift.setPower(0);
            }
        } else {
            robot.lift.setPower(0);
        }

        telemetry.addData("Lift Position:", robot.lift.getCurrentPosition());
        telemetry.update();
    }
}