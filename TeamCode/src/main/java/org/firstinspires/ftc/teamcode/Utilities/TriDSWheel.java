package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class TriDSWheel {

    DSWheel wheelA;

    DSWheel wheelB;

    DSWheel wheelC;

    RealMatrix rotationMatrix;

    //RealVector position;

    public TriDSWheel(DcMotor motorA1,DcMotor motorA2, DcMotor motorB1, DcMotor motorB2, DcMotor motorC1, DcMotor motorC2){
        wheelA = new DSWheel(motorA1,motorA2);
        wheelB = new DSWheel(motorB1,motorB2);
        wheelC = new DSWheel(motorC1,motorC2);
    }
    public void setOrientation(double degrees){

    }

    public void moveGlobalDirection(double x, double y){

    }
}
