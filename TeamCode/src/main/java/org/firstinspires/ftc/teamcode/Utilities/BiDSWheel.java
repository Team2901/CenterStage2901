package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.DcMotor;

public class BiDSWheel {

    private int TICKS_PER_REV;
    private int GEAR_RATIO;

    DSWheel wheelA;

    DSWheel wheelB;

    RotationMatrix2 rotationMatrix;

    Vector2 position;

    public BiDSWheel(DcMotor motorA1, DcMotor motorA2, DcMotor motorB1, DcMotor motorB2, DcMotor motorC1, DcMotor motorC2){
        wheelA = new DSWheel(motorA1,motorA2);
        wheelB = new DSWheel(motorB1,motorB2);
        rotationMatrix = new RotationMatrix2();
        position = new Vector2();
    }

    public void setOrientation(double degrees){
        rotationMatrix.setFromAngle(degrees);
    }

    public void setPosition(double x, double y){
        position.x = x;
        position.y = y;
    }

    public void setWheelDirections(Vector2 direction){
        wheelA.setTargetVector(direction);
        wheelB.setTargetVector(direction);
    }

    public void moveGlobalDirection(double x, double y){

    }
    public void setTicksPerRevolution(int ticksPerRevolution){
        TICKS_PER_REV = ticksPerRevolution;
        this.wheelA.setTicksPerRevolution(TICKS_PER_REV);
        this.wheelB.setTicksPerRevolution(TICKS_PER_REV);
    }
    public void setGearRatio(int gearRatio){
        GEAR_RATIO = gearRatio;
        this.wheelA.setGearRatio(gearRatio);
        this.wheelB.setGearRatio(gearRatio);
    }
}
