package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DSWheel {

    private int TICKS_PER_REV;

    //motor revolution for every 1 wheel revolution
    private int GEAR_RATIO;
    DcMotor motor1;
    DcMotor motor2;

    RotationMatrix2 rotationMatrix;

    Vector2 targetVector;

    public DSWheel(DcMotor motor1, DcMotor motor2){
        this.motor1 = motor1;
        this.motor2 = motor2;
        rotationMatrix = new RotationMatrix2();
        targetVector = new Vector2();

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor1.setPower(0.0);
        motor2.setPower(0.0);
    }

    public void update(double ms){

    }

    public void rotateToTarget() {
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double angleProximity = rotationMatrix.multiply(0, 1).dot(targetVector);

        if (angleProximity < 0) {
            //flip rotation matrix
            rotationMatrix = rotationMatrix.multiply(new RotationMatrix2(Math.PI));
        }

        RotationMatrix2 angleMatrix = new RotationMatrix2().lookAt(targetVector).inverse().multiply(rotationMatrix);

        double radiansToTurn = angleMatrix.getAngle();
        int tickDifference = (int) (radiansToTurn * GEAR_RATIO * TICKS_PER_REV);
        motor1.setTargetPosition(motor1.getCurrentPosition() + tickDifference);
        motor2.setTargetPosition(motor2.getCurrentPosition() - tickDifference);
        motor1.setPower(0.5);
        motor2.setPower(0.5);
        while (motor1.isBusy() && motor2.isBusy()) {

        }
        motor1.setPower(0.0);
        motor2.setPower(0.0);
    }

    public void runWheel(){
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setTargetVector(double x, double y){
        targetVector.x = x;
        targetVector.y = y;
    }
    public void setTargetVector(Vector2 vector){
        targetVector.x = vector.x;
        targetVector.y = vector.y;
    }

    public void setCurrentOrientation(double angle){
        //possibly neccesary depending on how motors work
        rotationMatrix.setFromAngle(angle);
    }
    public void setTicksPerRevolution(int ticksPerRevolution){
        TICKS_PER_REV = ticksPerRevolution;
    }

    public void setGearRatio(int ratio){
        GEAR_RATIO = ratio;
    }
}
