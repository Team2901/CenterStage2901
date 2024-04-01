package org.firstinspires.ftc.teamcode.gearball;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//import org.bubblecloud.vecmath.FastMath;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;

import java.util.Arrays;
import java.util.Vector;

import info.laht.threekt.math.Euler;
import info.laht.threekt.math.EulerOrder;
import info.laht.threekt.math.Matrix3;
import info.laht.threekt.math.Matrix4;
import info.laht.threekt.math.Quaternion;
import info.laht.threekt.math.Vector3;

//import com.
public class GearBallHardware {
    private static final double DEG_TO_RAD = Math.PI / 180;
    private static final double TWO_PI = 2 * Math.PI;
    Telemetry telemetry;
    ImprovedGamepad gamepad;

    CRServo axonServo0;
    CRServo axonServo1;
    CRServo axonServo2;
    CRServo axonServo3;

    GBMPGear gearA;
    GBMPGear gearB;

    Vector3 currentRotation = new Vector3(0, 0, 0);

    public void init(HardwareMap hardwareMap) {
        axonServo0 = hardwareMap.crservo.get("axservo0");
        axonServo1 = hardwareMap.crservo.get("axservo1");
        axonServo2 = hardwareMap.crservo.get("axservo2");
        axonServo3 = hardwareMap.crservo.get("axservo3");

        gearA = new GBMPGear(axonServo0, axonServo1, hardwareMap.analogInput.get("axservopos0"), hardwareMap.analogInput.get("axservopos1"));
        gearB = new GBMPGear(axonServo2, axonServo3, hardwareMap.analogInput.get("axservopos2"), hardwareMap.analogInput.get("axservopos3"));

//        motorAngle_r_A = Math.PI;
//        motorAngle_p_A = Math.PI / 2;
//        motorAngle_r_B = Math.PI;
//        motorAngle_p_B = Math.PI / 2;

        motorAngle_r_A = Math.PI / 4;
        motorAngle_p_A = 0;
        motorAngle_r_B = Math.PI / 2;
        motorAngle_p_B = Math.PI;

//        motorAngle_r_A = 0;
//        motorAngle_p_A = 0;
//        motorAngle_r_B = 0;
//        motorAngle_p_B = 0;

        gearA.roll = motorAngle_r_A;
        gearA.pitch = motorAngle_p_A;
        gearB.roll = motorAngle_r_B;
        gearB.pitch = motorAngle_p_A;
    }

    int steps = 20;

    Quaternion currentQuat = new Quaternion(0, 0, 0, 1);
    Quaternion targetQuat = new Quaternion();
    Quaternion slerpedQuat = new Quaternion();
    Vector3 lookAt = new Vector3();
    Euler targetEuler = new Euler(0,0,0,EulerOrder.XYZ);
    Vector3 targetEulerVector = new Vector3();


    public void update(double stickX, double stickY) {
//        currentQuat.fromAngles(currentRotation.toArray(new float[3]));
//        if (stickX  stickY != 0) {
        targetEuler.setX(targetEuler.getX() + (float) ((targetEulerVector.x + stickX * 0.1) % TWO_PI));
        targetEuler.setY(targetEuler.getY() + (float) ((targetEulerVector.y + stickY * 0.1) % TWO_PI));
        targetEuler.setX((float) (Math.sin(stickX * Math.PI / 2)));
        targetEuler.setY((float) (Math.sin(stickY * Math.PI / 2)));
//        lookAt = new Vector3f((float) Math.sin(stickX * Math.PI / 2), (float) Math.sin(stickY * Math.PI / 2), 1).normalize();
//        targetQuat.loadIdentity();
//        targetQuat.lookAt(lookAt, new Vector3f(0, 0, 1));
        targetQuat.setFromEuler(targetEuler);
        processA();
//        }
//        telemetry.addData("lookAt",lookAt);
//        telemetry.addData("targetQuat",targetQuat);
//        telemetry.addData("angles", Arrays.toString(targetQuat.toAngles(new float[3])));
//        telemetry.update();
        currentQuat.copy(targetQuat);
    }

//    private void processA(Vector3f targetRotation) {
//        targetQuat.fromAngles(targetRotation.toArray(new float[3]));
//        for (float i = 0; i < 1; i += 0.05) {
//            slerpedQuat = new Quaternion().slerp(currentQuat, targetQuat, i);
//            processB();
//        }
//        currentQuat.set(targetQuat);
//    }

    //assumes quaternion has been updated
    private void processA() {
//        processB();
//        return;
//        telemetry.addData("targetQuat", targetQuat);
//        telemetry.update();
//        sleep(500);
        for (float i = 0; i <= 1; i += 1.0 / steps) {
////            telemetry.addData("i", i);
////            telemetry.update();
            slerpedQuat.copy(currentQuat).slerp(targetQuat, i);
//            //get euler angles in yaw, roll, pitch
//            float[] targetEulerArray =  slerpedQuat.toAngles(new float[3]);
//            targetEulerVector.set(targetEulerArray[1],targetEulerArray[2],targetEulerArray[0]);
            processB();
        }
        gearA.stop();
        gearB.stop();
    }

    public double motorAngle_r_A;
    public double motorAngle_p_A;
    public double motorAngle_r_B;
    public double motorAngle_p_B;

    //Module A
    double alpha_A = Math.PI / 2;
    double beta_A = 0;
    double gamma_A = -Math.PI / 4;

    //Module B
    double alpha_B = -Math.PI / 2;
    double beta_B = 0;
    double gamma_B = Math.PI * 3 / 4;

    //create mechanical constant rotation matrices
    Matrix3 mechanicalConst_A = rotationMatrix(alpha_A, beta_A, gamma_A);
    Matrix3 mechanicalConst_B = rotationMatrix(alpha_B, beta_B, gamma_B);
    Matrix3 mechanicalConstInverse_A = mechanicalConst_A.getInverse(new Matrix3());
    Matrix3 mechanicalConstInverse_B = mechanicalConst_B.getInverse(new Matrix3());

    public void calculateMPGear() {

    }

    public void processB() {

        Matrix3 targetOrientation = rotationMatrixFromQuat(slerpedQuat);

        Vector3 jA = new Vector3(1, 0, 0);
        Vector3 jB = new Vector3(0, 1, 0);

        //inverse kinematics stuff
        jA.applyMatrix3(mechanicalConstInverse_A.clone().multiply(targetOrientation));
        jB.applyMatrix3(mechanicalConstInverse_B.clone().multiply(targetOrientation));

        if (jA.y == 0 && jA.z == 0) {
            jA.y = (float) 0.01;
            jA.z = (float) 0.01;
        }
        ;
        if (jB.y == 0 && jB.z == 0) {
            jB.y = (float) 0.01;
            jB.z = (float) 0.01;
        }
        ;

        //get roll
        double roll_A = Math.atan2((double) jA.y, (double) jA.z);
        double roll_B = Math.atan2((double) jB.y, (double) jB.z);

        //get pitch
        double pitch_A = 2 * Math.acos((double) jA.x);
        double pitch_B = 2 * Math.acos((double) jB.x);

        telemetry.addData("lookAt", lookAt);
        telemetry.addData("angles", targetEuler);
        telemetry.addData("slerpedQuat", slerpedQuat.toString());
//        telemetry.addData("targetOrientation", targetOrientation);
//        telemetry.addData("mechanicalConstInverse_A", mechanicalConstInverse_A);
//        telemetry.addData("mechanicalConstInverse_B", mechanicalConstInverse_B);


        pitch_A = cleanAngle(pitch_A);
        roll_A = cleanAngle(roll_A);
        pitch_B = cleanAngle(pitch_B);
        roll_B = cleanAngle(roll_B);

        double dPitch_A = pitch_A - motorAngle_p_A;
        double dRoll_A = roll_A - motorAngle_r_A;
        double dPitch_B = pitch_B - motorAngle_p_B;
        double dRoll_B = roll_B - motorAngle_r_B;

        telemetry.addData("jA", jA);
        telemetry.addData("jB", jB);
        if (dRoll_A != 0 || dRoll_B != 0 || dPitch_A != 0 || dPitch_B != 0) {
            telemetry.addData("D_roll_A", dRoll_A);
            telemetry.addData("D_pitch_A", dPitch_A);
            telemetry.addData("D_roll_B", dRoll_B);
            telemetry.addData("D_pitch_B", dPitch_B);
        }

        telemetry.addData("roll_A", roll_A);
        telemetry.addData("pitch_A", pitch_A);
        telemetry.addData("roll_B", roll_B);
        telemetry.addData("pitch_B", pitch_B);

        gearA.setTargetOrientation(pitch_A, roll_A);
        gearB.setTargetOrientation(pitch_B, roll_B);

        telemetry.update();
//        sleep(1000);
        processC();

        motorAngle_r_A = roll_A;
        motorAngle_p_A = pitch_A;
        motorAngle_r_B = roll_B;
        motorAngle_p_B = pitch_B;
    }

    private void processC() {
//        telemetry.addData("A",null);
//        telemetry.addData("B",null);

        boolean aDone = false;
        boolean bDone = false;
//        bDone = true;

        while (!aDone || !bDone) {
            if (gamepad != null && gamepad.left_stick_button.isInitialPress()) break;
            aDone = gearA.moveToTargetOrientation();
            bDone = gearB.moveToTargetOrientation();
//            telemetry.addData("ADone",aDone);
//            telemetry.addData("bDone",bDone);
            telemetry.update();
        }
    }

    private Matrix4 temp4x4 = new Matrix4();

    private Matrix3 rotationMatrix(double x, double y, double z) {
//        assert temp4x4 != null;
        if (temp4x4 == null) temp4x4 = new Matrix4();
        temp4x4.identity();
        temp4x4.makeRotationFromEuler(new Euler(x, y, z));
        return new Matrix3().setFromMatrix4(temp4x4.extractRotation(temp4x4));
    }

    private Matrix3 rotationMatrixFromQuat(Quaternion quat){
        temp4x4.identity();
        temp4x4.compose(new Vector3(0,0,0),quat,new Vector3(1,1,1));
        return new Matrix3().setFromMatrix4(temp4x4.extractRotation(temp4x4));
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.gearA.setTelemetry(telemetry);
        this.gearB.setTelemetry(telemetry);
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        this.telemetry = dashboard.getTelemetry();
    }

    public void setGamepad(ImprovedGamepad gamepad) {
        this.gamepad = gamepad;
        this.gearA.setGamepad(gamepad);
        this.gearB.setGamepad(gamepad);
    }

    private void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    private double cleanAngle(double angle) {
        while (angle < 0) {
            angle += Math.PI * 2;
        }
        return angle % (Math.PI * 2);
    }
}