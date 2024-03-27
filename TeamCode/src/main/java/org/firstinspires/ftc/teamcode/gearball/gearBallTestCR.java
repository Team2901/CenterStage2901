package org.firstinspires.ftc.teamcode.gearball;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;

@Autonomous(name = "gearBallTestCR", group = "AAAtest")
public class gearBallTestCR extends LinearOpMode {

    double DRIVE_RATIO = 48.0 / 40 * 48 / 60;

    public boolean isStopped = false;

    CRServo test0;
    CRServo test1;
    AnalogInput inputTest0;
    AnalogInput inputTest1;
    AnalogInput inputTest2;
    AnalogInput inputTest3;

    ImprovedGamepad impGamepad1;
    ImprovedGamepad impGamepad2;

    @Override
    public void runOpMode() throws InterruptedException {
        test0 = this.hardwareMap.crservo.get("axservo0");
        test1 = this.hardwareMap.crservo.get("axservo1");
        inputTest0 = this.hardwareMap.analogInput.get("axservopos0");
        inputTest1 = this.hardwareMap.analogInput.get("axservopos1");
        inputTest2 = this.hardwareMap.analogInput.get("axservopos2");
        inputTest3 = this.hardwareMap.analogInput.get("axservopos3");
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

        test1Voltage = inputTest1.getVoltage() / inputTest1.getMaxVoltage();
        test1Angle = test1Voltage * Math.PI * 2;
    }

    double increment = 100 / 225.0;

    double test1Angle;
    double test1Voltage;

    void _loop() throws InterruptedException {
        impGamepad1.update();
        impGamepad2.update();
        updateCRServoAngle();
        Thread.sleep(10);
        //servoback pos && servofor neg = clockwise
        //servoback neg && servofor pos = cclockwise
        //servoback pos = pitchup
        //servoback neg = pitchdown
        double cr0power = 0;
        double cr1power = 0;
        if (impGamepad1.x.getValue()) {
            cr0power = increment;
        }
        if (impGamepad1.b.getValue()) {
            cr0power = -increment;
        }
        if (impGamepad1.y.getValue()) {
            cr1power = increment;
        }
        if (impGamepad1.a.getValue()) {
            cr1power = -increment;
        }
        if (impGamepad1.dpad_right.getValue()) {
            cr1power = increment;
            cr0power = -increment;
        }
        if (impGamepad1.dpad_left.getValue()) {
            cr1power = -increment;
            cr0power = +increment;
        }
        if (impGamepad1.dpad_up.getValue()) {
            cr1power = increment;
        }
        if (impGamepad1.dpad_down.getValue()) {
            cr1power = -increment;
        }
        if (impGamepad1.right_trigger.getValue() > 0.5) {
            pitchMPGear(Math.PI * 2);
        }
        test0.getController().getPwmStatus();
        test0.setPower(cr0power);
        test1.setPower(cr1power);
        telemetry.addData("actual0", inputTest0.getVoltage() / inputTest0.getMaxVoltage());
        telemetry.addData("actual1", inputTest1.getVoltage() / inputTest1.getMaxVoltage());
        telemetry.addData("actual1angle", test1Angle * 180 / Math.PI);
        telemetry.addData("actual2", inputTest2.getVoltage() / inputTest2.getMaxVoltage());
        telemetry.addData("actual3", inputTest3.getVoltage() / inputTest3.getMaxVoltage());
        telemetry.addData("try0", cr0power);
        telemetry.addData("try1", cr1power);
        telemetry.addData("pwn status", test0.getController().getPwmStatus());
        telemetry.addData("port0", test0.getPortNumber());
        telemetry.addData("port1", test1.getPortNumber());
        telemetry.update();
//        if(test0.getPosition() == 1.0 || test1.getPosition() == 1.0)isStopped = true;
    }

    void updateCRServoAngle() {
        double rawVoltage = inputTest1.getVoltage() / inputTest1.getMaxVoltage();
        double dx = rawVoltage - test1Voltage;
        telemetry.addData("dx", dx);
        if (Math.abs(dx) >= 0.6) {
            //went from 1 -> 0 or 0 -> 1
            if (test1Voltage >= 0.6) {
                dx++;
            } else if (test1Voltage <= 0.2) {
                dx--;
            }
        }
        double dtheta = dx * Math.PI * 2;
        telemetry.addData("dtheta", dtheta);
        test1Angle = test1Angle + dtheta;
        test1Voltage = rawVoltage;
    }

    double MAXPWR = 0.2;

    void pitchMPGear(double angle) throws InterruptedException {
        angle *= DRIVE_RATIO;
        updateCRServoAngle();
        double startAngle = test1Angle;
        double targetAngle = startAngle + angle;
        double distFromTarget = targetAngle - test1Angle;
//        telemetry.addData("currentAngle", test1Angle);
//        telemetry.addData("targetAngle",targetAngle);
//        telemetry.addData("distancefromtarget",distFromTarget);
//        telemetry.update();
//        Thread.sleep(2000);
        while (Math.abs(distFromTarget) > 0.08) {
            updateCRServoAngle();
            distFromTarget = targetAngle - test1Angle;
            double power = Math.min(MAXPWR, Math.max(-MAXPWR, distFromTarget));
            test1.setPower(-power);
            telemetry.addData("currentAngle", test1Angle);
            telemetry.addData("targetAngle", targetAngle);
            telemetry.addData("distancefromtarget", distFromTarget);
            telemetry.addData("power", power);
            telemetry.update();
            Thread.sleep(3);
        }
//        test1Angle = targetAngle;
        test1.setPower(0);
    }
}
