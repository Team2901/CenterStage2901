package org.firstinspires.ftc.teamcode.gearball;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;

@Config
@TeleOp(name = "gearBallUse", group = "AAAtest")
public class gearBallTestUse extends LinearOpMode {
    public boolean isStopped = false;

    ImprovedGamepad impGamepad1;
    ImprovedGamepad impGamepad2;

    GBMPGear gearA;
    GBMPGear gearB;

    Telemetry telemetryD;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetryD = dashboard.getTelemetry();
//        new debugGraph();
        impGamepad1 = new ImprovedGamepad(gamepad1, new ElapsedTime(), "gamepad1");
        impGamepad2 = new ImprovedGamepad(gamepad2, new ElapsedTime(), "gamepad2");
        gearA = new GBMPGear(this.hardwareMap.crservo.get("axservo0"), this.hardwareMap.crservo.get("axservo1"), this.hardwareMap.analogInput.get("axservopos0"), this.hardwareMap.analogInput.get("axservopos1"));
        gearB = new GBMPGear(this.hardwareMap.crservo.get("axservo2"), this.hardwareMap.crservo.get("axservo3"), this.hardwareMap.analogInput.get("axservopos2"), this.hardwareMap.analogInput.get("axservopos3"));
        gearA.setTelemetry(telemetry);
        gearA.setGamepad(impGamepad1);
        gearB.setTelemetry(telemetry);
        gearB.setGamepad(impGamepad1);
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

    double kP = 1;
    double kI = 0.5;
    double kD = 0;
    double kV = 0;
    double kA = 0;
    double kStatic = 0;
    int kType;

    void _loop() {
        impGamepad1.update();
        impGamepad2.update();
        printAngles();
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
            gearA.setTargetOrientation(Math.PI * 2 * factor, Math.PI * 2 * factor);
//            gearB.setTargetOrientation(Math.PI * 2 * factor, Math.PI * 2 * factor);
            boolean aDone = false;
//            boolean bDone = false;

            while (!aDone /*|| !bDone*/) {
                if (impGamepad1.left_stick_button.isInitialPress()) break;
                aDone = gearA.moveToTargetOrientation();
//                bDone = gearB.moveToTargetOrientation();
                printAngles();
            }
        }
        if (impGamepad1.left_trigger.getValue() > 0.5) {
            gearA.pitchMPGear(Math.PI * 2 * -factor);
            gearB.pitchMPGear(Math.PI * 2 * -factor);

        }
        if (impGamepad1.right_bumper.getValue()) {
            gearA.rollMPGear(Math.PI * 2 * factor);
            gearB.rollMPGear(Math.PI * 2 * factor);
        }
        if (impGamepad1.left_bumper.getValue()) {
            gearA.rollMPGear(Math.PI * 2 * -factor);
            gearB.rollMPGear(Math.PI * 2 * -factor);
        }
        if (impGamepad1.right_stick_button.isInitialPress()) {
            kType = (kType + 1) % 6;
        }
        if (impGamepad1.right_stick_x.getValue() != 0 || impGamepad1.right_stick_y.getValue() != 0) {
            double change = Math.copySign(0.01, impGamepad1.right_stick_x.getRawValue());
            if (kType == 0) {
                kP += change;
            } else if (kType == 1) {
                kI += change;
            } else if (kType == 2) {
                kD += change;
            } else if (kType == 3) {
                kV += change;
            } else if (kType == 4) {
                kA += change;
            } else if (kType == 5) {
                kStatic += change;
            }
            gearA.setPID(kP, kI, kD, kV, kA, kStatic);
            gearB.setPID(kP, kI, kD, kV, kA, kStatic);
            sleep(50);
        }
        telemetry.addData("factor", factor);
        telemetry.addData("servoangle0", gearA.servoB.angle);
        telemetry.addData("servoangle1", gearA.servoF.angle);
        telemetry.addData("servoangle0target", gearA.servoB.targetAngle);
        telemetry.addData("servoangle1target", gearA.servoF.targetAngle);
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.addData("kV", kV);
        telemetry.addData("kA", kA);
        telemetry.addData("kStatic", kStatic);
        telemetry.update();
    }

    public void printAngles() {
        gearA.servoF.updateAngle();
        gearA.servoB.updateAngle();
        gearB.servoF.updateAngle();
        gearB.servoB.updateAngle();
        telemetryD.addData("GearA_F", gearA.servoF.angle);
        telemetryD.addData("GearA_B", gearA.servoB.angle);
        telemetryD.addData("GearB_F", gearB.servoF.angle);
        telemetryD.addData("GearB_B", gearB.servoB.angle);
        telemetryD.addData("GearA_F_target", gearA.servoF.targetAngle);
        telemetryD.addData("GearA_F_previous", gearA.servoF.previousAngle);
        telemetryD.addData("GearA_B_target", gearA.servoB.targetAngle);
        telemetryD.addData("GearA_B_previous", gearA.servoB.previousAngle);
        telemetryD.update();
    }
}
