package org.firstinspires.ftc.teamcode.gearball;

//import com.image.charts.ImageCharts;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
//import org.knowm.xchart.BitmapEncoder;
//
//import java.io.IOException;


public class GBMPGear {
    private final double PITCH_RATIO = 48.0 / 40 * 48 / 60;
    private final double ROLL_RATIO = 48.0 / 40;

    private final double MAXPWR = 0.3;
    private final double MINPWR = 0.2;
    private final double MINDST = 0.03;
    ImprovedGamepad gamepad;

    //servo forwards and backwards
    public GBCRServo servoB;
    public GBCRServo servoF;

    public double pitch = 0;
    public double roll = 0;

    Telemetry telemetryD;

    public GBMPGear(CRServo servo0, CRServo servo1, AnalogInput input0, AnalogInput input1) {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetryD = dashboard.getTelemetry();
        servoB = new GBCRServo(servo0, input0);
        servoF = new GBCRServo(servo1, input1);
    }

    Telemetry telemetry;

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void setGamepad(ImprovedGamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void setPID(double kP, double kI, double kD, double kV, double kA, double kStatic) {
        servoB.setPIDCoefficients(kP, kI, kD, kV, kA, kStatic);
        servoF.setPIDCoefficients(kP, kI, kD, kV, kA, kStatic);
    }

    public void pitchMPGear(double theta) {
        servoF.updateAngle();
        servoF.setTarget(theta * PITCH_RATIO);
        servoB.servo.setPower(0);
        while (!servoF.moveToTarget()) {
            gamepad.update();
            if (gamepad != null && gamepad.left_stick_button.getValue()) break;
            telemetry.addData("servoF target", servoF.targetAngle);
            telemetry.addData("servoF angle", servoF.angle);
            telemetry.addData("servoF power", servoF.power);
            telemetry.addData("servoF dvolts", servoF.dvolts);
            telemetryD.addData("servoF dvolts", servoF.dvolts);
            telemetryD.addData("x", servoF.angle);
            telemetryD.addData("t", servoF.targetAngle);
            telemetryD.update();
        }
        servoF.servo.setPower(0);
        servoB.servo.setPower(0);
        this.pitch += theta;
    }

//    public void rollMPGear(double theta) {
//        updateRollAngle();
//        double targetAngle = roll + theta;
//        double distFromTarget = 1;
//        while (Math.abs(distFromTarget) > 0.03) {
//            updateRollAngle();
//            distFromTarget = targetAngle - roll;
//            double power = Math.copySign(MAXPWR, distFromTarget * 2.5);//Math.min(MAXPWR, Math.max(-MAXPWR, distFromTarget));
//            telemetry.addData("current", roll * 180 / Math.PI);
//            telemetry.addData("target", targetAngle * 180 / Math.PI);
//            telemetry.addData("distFromTarget", distFromTarget * 180 / Math.PI);
//            telemetry.addData("power", power);
//            telemetry.addData("roll ratio", ROLL_RATIO);
//            telemetry.update();
//            servoF.servo.setPower(power);
//            servoB.servo.setPower(-power);
//            sleep(5);
//        }
//        servoF.servo.setPower(0);
//        servoB.servo.setPower(0);
//        sleep(2000);
//    }

    private void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    private void updatePitchAngle() {
        servoF.updateAngle();
        pitch += servoF.dtheta * PITCH_RATIO;
    }

    private void updateRollAngle() {
        servoF.updateAngle();
        servoB.updateAngle();
        roll += servoB.dtheta * ROLL_RATIO;
//        roll += Math.copySign((Math.abs(servoF.dtheta) + Math.abs(servoB.dtheta)), servoB.dtheta) * ROLL_RATIO;
    }

    //    public void pitchMPGear(double theta) {
//        servoF.updateAngle();
//        servoF.setTarget(theta * DRIVE_RATIO);
//        while (!servoF.moveToTarget()) {
//        }
//        //servoF.rotate(theta * DRIVE_RATIO);
//    }
//
    public void rollMPGear(double theta) {
        servoB.updateAngle();
        servoF.updateAngle();
        servoB.setTarget(theta * ROLL_RATIO);
        servoF.setTarget(-theta * ROLL_RATIO);
        boolean bDone = false;
        boolean fDone = false;

        while (!bDone || !fDone) {
            gamepad.update();
            if (gamepad != null && gamepad.left_stick_button.isInitialPress()) break;
            telemetry.addData("servoB target", servoB.targetAngle);
            telemetry.addData("servoB angle", servoB.angle);
            telemetry.addData("servoB power", servoB.power);
            telemetry.addData("servoF target", servoF.targetAngle);
            telemetry.addData("servoF angle", servoF.angle);
            telemetry.addData("servoF power", servoF.power);
            telemetry.update();
            telemetryD.addData("F", servoF.angle);
            telemetryD.addData("tF", servoF.targetAngle);
            telemetryD.addData("vF", servoF.dvolts);
            telemetryD.addData("B", servoB.angle);
            telemetryD.addData("tB", servoB.targetAngle);
            telemetryD.addData("vB", servoB.dvolts);
            telemetryD.update();
            if (!bDone) bDone = servoB.moveToTarget();
            if (!fDone) fDone = servoF.moveToTarget();

//            sleep(5);
        }
        servoF.servo.setPower(0);
        servoB.servo.setPower(0);
        this.roll += theta;
//        servoF.rotate(theta * DRIVE_RATIO);
//        servoB.rotate(-theta * DRIVE_RATIO);

    }

    public void orientateTo(double pitch, double roll) {
        double dPitch = pitch - this.pitch;
        double dRoll = roll - this.roll;

        this.pitchMPGear(dPitch);
        this.rollMPGear(dRoll);

        this.pitch += pitch;
        this.roll += roll;
    }

    double dPitch;
    double dRoll;

    double targetPitch;
    double targetRoll;


    int orientationStatus = 0;

    public void setTargetOrientation(double pitch, double roll) {
        targetPitch = pitch;
        targetRoll = roll;
        dPitch = targetPitch - this.pitch;
        dRoll = targetRoll - this.roll;
        telemetryD.addData("dPitch", dPitch);
//        telemetry.addData("dPitch", dPitch);
//        telemetry.addData("dRoll", dRoll);
        telemetry.update();
        if (Math.abs(dPitch) < MINDST) dPitch = 0;
        if (Math.abs(dRoll) < MINDST) dRoll = 0;
        telemetryD.addData("aPitch", dPitch);

        orientationStatus = 0;

        servoB.updateAngle();
        servoF.updateAngle();

        double servoFTarget = dPitch * PITCH_RATIO; //+ dRoll * ROLL_RATIO;
        double servoBTarget = -dRoll * ROLL_RATIO;
        double smallest = Math.min(Math.abs(servoFTarget), Math.abs(servoBTarget));
        double servoFVel = Math.abs(servoFTarget) / smallest * MINPWR;
        double servoBVel = Math.abs(servoBTarget) / smallest * MINPWR;
        if (servoFTarget == 0 && servoBTarget != 0) {
            servoBVel = MINPWR;
        } else if (servoBTarget == 0 && servoFTarget != 0) {
            servoFVel = MINPWR;
        }
//        else if(servoBTarget == 0 && servoFTarget == 0){
//
//        }

//        telemetry.addData("servoFVel", servoFVel);
//        telemetry.addData("servoBVel", servoBVel);
//        telemetry.update();

        //can't move less than 0.05
//        if (Math.abs(dPitch) > 0.05) {
        servoF.setTargetAbsolute(servoFTarget, servoFVel);
//        } else {
//            servoF.setTargetAbsolute(0, MINPWR);
//        }
//        if (Math.abs(dRoll) > 0.05) {
        servoB.setTargetAbsolute(servoBTarget, servoBVel);
//        } else {
//            servoB.setTargetAbsolute(0, MINPWR);
//        }
        telemetryD.addData("pitch", this.pitch);
        telemetryD.addData("targetPitch", targetPitch);
        telemetryD.addData("F", servoF.angle);
        telemetryD.addData("tF", servoF.targetAngle);
        telemetryD.addData("dF", servoF.targetAngle-servoF.angle);
        telemetryD.addData("pF", servoF.power);
        telemetryD.addData("mF", servoFTarget);

        telemetryD.addData("B", servoB.angle);
        telemetryD.addData("tB", servoB.targetAngle);
        telemetryD.update();

//        telemetry.addData("dPitch",dPitch);
//        telemetry.addData("dRoll",dRoll);
//        telemetry.addData("targetF",servoF.targetAngle);
//        telemetry.addData("angleF",servoF.angle);
//        telemetry.addData("targetB",servoB.targetAngle);
//        telemetry.addData("angleB",servoB.angle);
//        telemetry.update();
//        sleep(500);
    }

    public boolean moveToTargetOrientation() {
        telemetryD.addData("F", servoF.angle);
        telemetryD.addData("tF", servoF.targetAngle);
        telemetryD.addData("B", servoB.angle);
        telemetryD.addData("tB", servoB.targetAngle);
        telemetryD.update();
//        if (orientationStatus == 0) {
//            boolean fDone = servoF.moveToTarget();
//            servoB.servo.setPower(0);
//            if (fDone) {
//                orientationStatus++;
//            }
//        } else if (orientationStatus == 1) {
//            servoB.setTarget(dRoll / ROLL_RATIO);
//            servoF.setTarget(-dRoll / ROLL_RATIO);
//            orientationStatus++;
//        }
//        if (orientationStatus == 2) {
        boolean bDone = servoB.moveTowardTarget();
        boolean fDone = servoF.moveTowardTarget();
        if (bDone && fDone) {
            if (orientationStatus == 0) {
                servoF.servo.setPower(0);
                servoB.servo.setPower(0);
                if (Math.abs(dPitch) > MINDST) pitch = targetPitch;
                if (Math.abs(dRoll) > MINDST) roll = targetRoll;
//                telemetry.addData("finished", null);
//                telemetry.update();
                orientationStatus++;
            }

            return true;
        }
//        }
//        if (orientationStatus == 3) {
//            return true;
//        }
//        telemetry.addData("Fdvolts",servoF.dvolts);
//        telemetry.addData("Bdvolts",servoF.dvolts);
//        telemetry.update();
        return false;
    }

    //    public void setTargetOrientation(double pitch, double roll) {
//        dPitch = pitch - this.pitch;
//        dRoll = roll - this.roll;
//        orientationStatus = 0;
//
//        servoB.updateAngle();
//        servoF.updateAngle();
//
//        servoF.setTarget(dPitch * PITCH_RATIO);
//    }
//    public boolean moveToTargetOrientation() {
//
//        if (orientationStatus == 0) {
//            boolean fDone = servoF.moveToTarget();
//            servoB.servo.setPower(0);
//            if (fDone) {
//                orientationStatus++;
//            }
//        } else if (orientationStatus == 1) {
//            servoB.setTarget(dRoll / ROLL_RATIO);
//            servoF.setTarget(-dRoll / ROLL_RATIO);
//            orientationStatus++;
//        }
//        if (orientationStatus == 2) {
//            boolean bDone = servoB.moveToTarget();
//            boolean fDone = servoF.moveToTarget();
//            if (bDone && fDone) {
//                servoF.servo.setPower(0);
//                servoB.servo.setPower(0);
//                this.pitch += dPitch;
//                this.roll += dRoll;
//                orientationStatus++;
//                return true;
//            }
//        }
//        if (orientationStatus == 3) {
//            return true;
//        }
//        telemetryD.addData("F", servoF.angle);
//        telemetryD.addData("tF", servoF.targetAngle);
//        telemetryD.addData("B", servoB.angle);
//        telemetryD.addData("tB", servoB.targetAngle);
//        telemetryD.update();
//        return false;
//    }
    public void stop() {
        servoF.servo.setPower(0);
        servoB.servo.setPower(0);
    }
}
