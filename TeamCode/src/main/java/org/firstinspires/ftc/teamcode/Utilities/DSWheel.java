package org.firstinspires.ftc.teamcode.Utilities;
import android.opengl.Matrix;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DSWheel {
    DcMotor motor1;
    DcMotor motor2;

    RotationMatrix2 rotationMatrix;

    Vector2 targetVector;

    public DSWheel(DcMotor motor1, DcMotor motor2){
        this.motor1 = motor1;
        this.motor2 = motor2;
        rotationMatrix = new RotationMatrix2();
        targetVector = new Vector2();
    }

    public void rotateToTarget(){

    }

    public void runWheel(){

    }

    public void setTargetVector(double x, double y){
        targetVector.x = x;
        targetVector.y = y;

    }

    public void setCurrentOrientation(double angle){
        //possibly neccesary depending on how motors work
        rotationMatrix.setFromAngle(angle);
    }
}
