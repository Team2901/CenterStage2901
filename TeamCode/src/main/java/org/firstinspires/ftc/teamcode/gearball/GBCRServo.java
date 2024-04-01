package org.firstinspires.ftc.teamcode.gearball;

import android.net.wifi.p2p.WifiP2pManager;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
public class GBCRServo {
    public static double kD = 0;
    public static double kI = 0.005;
    public static double kP = 0.5;
    public static double kStatic = 0.1;

    private final double MAXPWR = 0.25;
    private final double MINPWR = 0.08;

    public double angle;
    public double voltage;
    public double dtheta;

    boolean stillInitializing = true;

    public final CRServo servo;
    private final AnalogInput analogInput;
    PIDFController pid = new PIDFController(new PIDCoefficients(kP, kI, kD), 0, 0, kStatic);

    public GBCRServo(CRServo servo, AnalogInput analogInput) {
        this.servo = servo;
        this.analogInput = analogInput;
        voltage = analogInput.getVoltage() / analogInput.getMaxVoltage();
        angle = voltage * Math.PI * 2;
        previousAngle = angle;
        this.updateAngle();
        this.servo.setPower(0);
    }

    double dvolts = 0;

//    double[] voltageSample = new double[3];
//    int voltageSamples = 0;

    public void updateAngle() {
        double newVoltage = analogInput.getVoltage() / analogInput.getMaxVoltage();
//        if(voltageSamples < voltageSample.length){
//            voltageSample[voltageSamples] = newVoltage;
//            voltageSamples++;
//            return;
//        }
//        voltageSamples = 0;
//        for(double voltage : voltageSample){
//            newVoltage = (newVoltage + voltage)/2;
//        }

        double dx = newVoltage - voltage;
        if (Math.abs(dx) >= 0.1) {
            //dx is normally around 0.01;
            //went from 1 -> 0 or 0 -> 1
            if (voltage >= 0.6) {
                dx++;
            } else if (voltage <= 0.4) {
                dx--;
            }
        }
        dtheta = dx * Math.PI * 2;
        angle += dtheta;
        voltage = newVoltage;
    }

    public void rotate(double theta) {
        updateAngle();
        double startAngle = angle;
        double targetAngle = startAngle + theta;
        double distFromTarget = targetAngle - angle;

        while (Math.abs(distFromTarget) > 0.03) {
            updateAngle();
            distFromTarget = targetAngle - angle;
            double power = Math.min(MAXPWR, Math.max(-MAXPWR, distFromTarget * 2.5));
            servo.setPower(-power);
        }
        servo.setPower(0);
    }

    public void setPIDCoefficients(double kP, double kI, double kD, double kV, double kA, double kStatic) {
        pid = new PIDFController(new PIDCoefficients(kP, kI, kD), kV, kA, kStatic);
    }

    double targetAngle;
    double previousAngle;
    double targetVelocity;

    /**
     * Adds the input rotation and current angle to set the target angle
     *
     * @param theta angle
     */
    public void setTarget(double theta) {
        targetAngle = angle + theta;
        pid.reset();
        pid.setTargetPosition(targetAngle);
        pid.setOutputBounds(-MAXPWR, MAXPWR);
    }

    public void setTargetAbsolute(double theta, double velocity) {
        checkInit();
        targetAngle = previousAngle + theta;
        targetVelocity = velocity;
        //already overshoot
//        if(Math.signum(targetAngle - angle) == Math.signum(theta)) {
//            targetAngle = previousAngle;
//        };
//        pid.reset();
//        pid.setTargetPosition(targetAngle);
//        pid.setOutputBounds(-MAXPWR, MAXPWR);
//        pid.setTargetVelocity(Math.copySign(velocity, targetAngle - angle));
    }

    /**
     * Assumes that setTarget has been called
     * rotates toward target angle
     *
     * @return false if still rotating, false if target not reached
     */
    public double power = MAXPWR;

    public boolean moveToTarget() {
        updateAngle();
        double diff = targetAngle - angle;
        if (Math.abs(diff) <= 0.08) {
            servo.setPower(0);
            return true;
        }
//        power = pid.update(angle);
        power = Math.copySign(targetVelocity, diff);//Math.min(MAXPWR, Math.max(-MAXPWR, diff / 2 + Math.copySign(0.1, diff)));
        servo.setPower(-power);
//        double distFromTarget = targetAngle - angle;
//        if (Math.abs(distFromTarget) < 0.08) {
//            angle = targetAngle;
//            servo.setPower(0);
//            return true;
//        }
//        servo.setPower(-power);
        return false;
    }

    /**
     * can exceed target;
     *
     * @return
     */
    public boolean moveTowardTarget() {
        updateAngle();
        double diff = targetAngle - angle;
        if (Math.abs(diff) <= 0.03 || Math.signum(diff) != Math.signum(targetAngle - previousAngle)) {
            servo.setPower(0);
            previousAngle = targetAngle;
            return true;
        }
        power = Math.copySign(targetVelocity, diff);//Math.min(MAXPWR, Math.max(-MAXPWR, diff / 2 + Math.copySign(MINPWR, diff)));
        servo.setPower(-power);
        return false;
    }

    public void checkInit() {
        if (!stillInitializing) return;
        this.previousAngle = angle;
        stillInitializing = false;
    }
}
