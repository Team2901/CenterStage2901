package org.firstinspires.ftc.teamcode.gearball;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

public class GBCRServo {
    private final double MAXPWR = 0.6;

    public double angle;
    public double voltage;
    public double dtheta;

    public final CRServo servo;
    private final AnalogInput analogInput;
    PIDFController pid = new PIDFController(new PIDCoefficients(0.5, 0.1, 0.1));

    public GBCRServo(CRServo servo, AnalogInput analogInput) {
        this.servo = servo;
        this.analogInput = analogInput;
        this.updateAngle();

    }

    public void updateAngle() {
        double newVoltage = analogInput.getVoltage() / analogInput.getMaxVoltage();
        double dx = newVoltage - voltage;
        if (Math.abs(dx) >= 0.6) {
            //went from 1 -> 0 or 0 -> 1
            if (voltage >= 0.6) {
                dx++;
            } else if (voltage <= 0.2) {
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

    public void setPIDCoefficients(double kP,double kI,double kD){
        pid = new PIDFController(new PIDCoefficients(kP,kI,kD));
    }

    double targetAngle;

    /**
     * Adds the input rotation and current angle to set the target angle
     *
     * @param theta angle
     */
    public void setTarget(double theta) {
        targetAngle = angle + theta;
        pid.setTargetPosition(targetAngle);
        pid.setOutputBounds(-MAXPWR,MAXPWR);
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
        if (Math.abs(targetAngle - angle) <= 0.05) {
            servo.setPower(0);
            return true;
        }
        power = pid.update(angle);
        servo.setPower(-power);
//        double distFromTarget = targetAngle - angle;
//        if (Math.abs(distFromTarget) < 0.08) {
//            angle = targetAngle;
//            servo.setPower(0);
//            return true;
//        }
//        double power = Math.copySign(MAXPWR, distFromTarget);//Math.min(MAXPWR, Math.max(-MAXPWR, distFromTarget));
//        servo.setPower(-power);
        return false;
    }
}
