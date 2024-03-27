package org.firstinspires.ftc.teamcode.gearball;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;

@TeleOp(name = "gearBallUse", group = "AAAtest")
public class gearBallTestUse extends LinearOpMode {
    public boolean isStopped = false;

    ImprovedGamepad impGamepad1;
    ImprovedGamepad impGamepad2;

    GBMPGear gearA;

    @Override
    public void runOpMode() throws InterruptedException {
//        new debugGraph();
        impGamepad1 = new ImprovedGamepad(gamepad1, new ElapsedTime(), "gamepad1");
        impGamepad2 = new ImprovedGamepad(gamepad2, new ElapsedTime(), "gamepad2");
        gearA = new GBMPGear(this.hardwareMap.crservo.get("axservo0"), this.hardwareMap.crservo.get("axservo1"), this.hardwareMap.analogInput.get("axservopos0"), this.hardwareMap.analogInput.get("axservopos1"));
        gearA.setTelemetry(telemetry);
        waitForStart();
        while (opModeIsActive()) {
            if (isStopped) {
                break;
            } else {
                _loop();
            }
        }
    }

    double factor = 1;

    double kP;
    double kI;
    double kD;
    int kType;

    void _loop() {
        impGamepad1.update();
        impGamepad2.update();
        if (impGamepad1.y.getValue()) {
            factor = 1;
        }
        if (impGamepad1.b.getValue()) {
            factor = 0.25;
        }
        if (impGamepad1.a.getValue()) {
            factor = 0.5;
        }
        if (impGamepad1.x.getValue()) {
            factor = 0.75;
        }
        if (impGamepad1.right_trigger.getValue() > 0.5) {
            gearA.pitchMPGear(Math.PI * 2 * factor);
        }
        if (impGamepad1.left_trigger.getValue() > 0.5) {
            gearA.pitchMPGear(Math.PI * 2 * -factor);
        }
        if (impGamepad1.right_bumper.getValue()) {
            gearA.rollMPGear(Math.PI * 2 * factor);
        }
        if (impGamepad1.left_bumper.getValue()) {
            gearA.rollMPGear(Math.PI * 2 * -factor);
        }
        if (impGamepad1.right_stick_button.isInitialPress()) {
            kType = (kType + 1) % 3;
        }
        if (impGamepad1.right_stick_x.getValue() != 0 || impGamepad1.right_stick_y.getValue() != 0) {
            double change = impGamepad1.right_stick_x.getValue()/50000;
            if(kType == 0){
                kP += change;
            }else if(kType == 1){
                kI += change;
            }else if(kType == 2){
                kD += change;
            }
            gearA.setPID(kP,kI,kD);
        }
        telemetry.addData("factor", factor);
        telemetry.addData("servoangle0", gearA.servoB.angle);
        telemetry.addData("servoangle1", gearA.servoF.angle);
        telemetry.addData("servoangle0target", gearA.servoB.targetAngle);
        telemetry.addData("servoangle1target", gearA.servoF.targetAngle);
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.update();
    }
}
