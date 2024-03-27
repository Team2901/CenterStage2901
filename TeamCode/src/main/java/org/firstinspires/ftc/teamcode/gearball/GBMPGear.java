package org.firstinspires.ftc.teamcode.gearball;

import android.os.Environment;

import androidx.core.math.MathUtils;

import com.image.charts.ImageCharts;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.knowm.xchart.BitmapEncoder;
//
//import java.io.IOException;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.security.InvalidKeyException;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;

public class GBMPGear {
    private final double PITCH_RATIO = 48.0 / 40 * 48 / 60;
    private final double ROLL_RATIO = 40.0 / 48;
    private final double MAXPWR = 0.2;

    //servo forwards and backwards
    public GBCRServo servoB;
    public GBCRServo servoF;

    public double pitch = 0;
    public double roll = 0;

    public GBMPGear(CRServo servo0, CRServo servo1, AnalogInput input0, AnalogInput input1) {
        servoB = new GBCRServo(servo0, input0);
        servoF = new GBCRServo(servo1, input1);
    }

    Telemetry telemetry;

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void setPID(double kP, double kI, double kD) {
        servoB.setPIDCoefficients(kP, kI, kD);
        servoF.setPIDCoefficients(kP, kI, kD);
    }

    public void pitchMPGear(double theta) {
        servoF.updateAngle();
        servoF.setTarget(theta * PITCH_RATIO);
        servoB.servo.setPower(0);
        ArrayList<Double> xData = new ArrayList<>();
        ArrayList<Double> yData = new ArrayList<>();
        double startTime = System.currentTimeMillis();
        while (!servoF.moveToTarget()) {
            telemetry.addData("servoF target", servoF.targetAngle);
            telemetry.addData("servoF angle", servoF.angle);
            telemetry.addData("servoF power", servoF.power);
            telemetry.update();
            xData.add(System.currentTimeMillis() - startTime);
            yData.add(servoF.angle);
        }
        String dataStr = "";
        for (double val : yData) {
            dataStr += Math.floor(val * 1000) / 1000 + ", ";
            ;
        }
        telemetry.addData("a", dataStr);
        telemetry.update();
        try (FileWriter out = new FileWriter(String.format("%s/FIRST/data/mylog.txt", Environment.getExternalStorageDirectory().getAbsolutePath())
        )) {
            out.write(dataStr);
            out.flush();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        sleep(1000);
//        try {
//                    new ImageCharts()
//                            .cht("ls") // vertical bar chart
//                            .chs("300x300")// 300px x 300px
//                            .chd("a:" + dataStr).toFile(String.format("%s/FIRST/data/imageish.png", Environment.getExternalStorageDirectory().getAbsolutePath()));
//        } catch (IOException | InvalidKeyException | NoSuchAlgorithmException e) {
//            throw new RuntimeException(e);
//        }
//        XYChart chart = QuickChart.getChart("Sample Chart", "X", "Y", "y(x)", xData, yData);
//        new SwingWrapper(chart).displayChart();
//        try {
//            BitmapEncoder.saveBitmap(chart, "./Sample_Chart", BitmapEncoder.BitmapFormat.PNG);
//        } catch (IOException e) {
//            throw new RuntimeException(e);
//        }

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
        servoB.setTarget(theta / ROLL_RATIO);
        servoF.setTarget(-theta / ROLL_RATIO);
        boolean bDone = false;
        boolean fDone = false;

        while (!bDone || !fDone) {
            telemetry.addData("servoB target", servoB.targetAngle);
            telemetry.addData("servoB angle", servoB.angle);
            telemetry.addData("servoB power", servoB.power);
            telemetry.addData("servoF target", servoF.targetAngle);
            telemetry.addData("servoF angle", servoF.angle);
            telemetry.addData("servoF power", servoF.power);
            telemetry.update();
            bDone = servoB.moveToTarget();
            fDone = servoF.moveToTarget();

//            sleep(5);
        }
//        servoF.rotate(theta * DRIVE_RATIO);
//        servoB.rotate(-theta * DRIVE_RATIO);

    }
}
