package org.firstinspires.ftc.teamcode.gearball;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;

@Autonomous(name = "gearBallTest", group = "AAAtest")
public class gearBallTest extends LinearOpMode {
    public boolean isStopped = false;

    Servo test0;
    Servo test1;
    AnalogInput inputTest0;

    ImprovedGamepad impGamepad1;
    ImprovedGamepad impGamepad2;

    @Override
    public void runOpMode() throws InterruptedException {
        test0 = this.hardwareMap.servo.get("axservo0");
        test1 = this.hardwareMap.servo.get("axservo1");
        inputTest0 = this.hardwareMap.analogInput.get("axservopos0");
        impGamepad1 = new ImprovedGamepad(gamepad1, new ElapsedTime(), "gamepad1");
        impGamepad2 = new ImprovedGamepad(gamepad2, new ElapsedTime(), "gamepad2");
//        test0.setPosition(0.5);
//        test1.setPosition(0.5);
        waitForStart();
        while (opModeIsActive()) {
            if (isStopped) {
                break;
            } else {
                _loop();
            }
        }
    }

    double increment = 1 / 225.0;

    void _loop() throws InterruptedException {
        impGamepad1.update();
        impGamepad2.update();
        Thread.sleep(10);
        //servoback pos && servofor neg = clockwise
        //servoback neg && servofor pos = cclockwise
        //servoback pos = pitchup
        //servoback neg = pitchdown

        if (impGamepad1.x.getValue()) {
            test0.setPosition(test0.getPosition() + increment);
        }
        if (impGamepad1.b.getValue()) {
            test0.setPosition(test0.getPosition() - increment);
        }
        if (impGamepad1.y.getValue()) {
            test1.setPosition(test1.getPosition() + increment);
        }
        if (impGamepad1.a.getValue()) {
            test1.setPosition(test1.getPosition() - increment);
        }
        if (impGamepad1.dpad_right.getValue()) {
            test1.setPosition(test1.getPosition() + increment);
            test0.setPosition(test0.getPosition() - increment);
        }
        if (impGamepad1.dpad_left.getValue()) {
            test1.setPosition(test1.getPosition() - increment);
            test0.setPosition(test0.getPosition() + increment);
        }
        if (impGamepad1.dpad_up.getValue()) {
            test1.setPosition(test1.getPosition() + increment);
        }
        if (impGamepad1.dpad_down.getValue()) {
            test1.setPosition(test1.getPosition() - increment);
        }
        telemetry.addData("analog value", inputTest0.getVoltage() / inputTest0.getMaxVoltage());
        telemetry.addData("actual0", test0.getPosition());
        telemetry.addData("actual1", test1.getPosition());
        telemetry.update();
//        if(test0.getPosition() == 1.0 || test1.getPosition() == 1.0)isStopped = true;
    }

    void rotateMPGear() {

    }
}
