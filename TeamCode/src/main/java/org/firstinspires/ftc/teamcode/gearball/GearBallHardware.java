package org.firstinspires.ftc.teamcode.gearball;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.bubblecloud.vecmath.Matrix3f;
import org.bubblecloud.vecmath.Matrix4f;
import org.bubblecloud.vecmath.Quaternion;
import org.bubblecloud.vecmath.Vector3f;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Vector;

//import com.
public class GearBallHardware {
    CRServo axonServo0;
    CRServo axonServo1;
    CRServo axonServo2;
    CRServo axonServo3;

    GBMPGear gearA;
    GBMPGear gearB;

    Vector3f currentRotation = new Vector3f(0, 0, 0);

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        axonServo0 = hardwareMap.crservo.get("axservo0");
        axonServo1 = hardwareMap.crservo.get("axservo1");
        axonServo2 = hardwareMap.crservo.get("axservo2");
        axonServo3 = hardwareMap.crservo.get("axservo3");

        gearA = new GBMPGear(axonServo0, axonServo1, hardwareMap.analogInput.get("axservopos0"), hardwareMap.analogInput.get("axservopos1"));
        gearB = new GBMPGear(axonServo2, axonServo3, hardwareMap.analogInput.get("axservopos2"), hardwareMap.analogInput.get("axservopos3"));

    }


    Quaternion currentQuat = new Quaternion();
    Quaternion targetQuat = new Quaternion();
    Quaternion slerpedQuat = new Quaternion();

    public void update(double stickX, double stickY){
//        currentQuat.fromAngles(currentRotation.toArray(new float[3]));
        if(stickX != 0 || stickY != 0){
            targetQuat.lookAt(new Vector3f(1,0,0),new Vector3f(0,1,0));
            processA();
        }
        currentQuat.set(targetQuat);
    }

    private void processA(Vector3f targetRotation) {
        targetQuat.fromAngles(targetRotation.toArray(new float[3]));
        for (float i = 0; i < 1; i += 0.05) {
            slerpedQuat = new Quaternion().slerp(currentQuat, targetQuat, i);
            processB();
        }
        currentQuat.set(targetQuat);
    }

    //assumes quaternion has been updated
    private void processA() {
        for (float i = 0; i < 1; i += 0.1) {
            slerpedQuat = new Quaternion().slerp(currentQuat, targetQuat, i);
            processB();
        }
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
    Matrix3f mechanicalConst_A = rotationMatrix(alpha_A, beta_A, gamma_A);
    Matrix3f mechanicalConst_B = rotationMatrix(alpha_B, beta_B, gamma_B);
    Matrix3f mechanicalConstInverse_A = mechanicalConst_A.invert();
    Matrix3f mechanicalConstInverse_B = mechanicalConst_B.invert();

    public void calculateMPGear() {

    }

    public void processB() {
        //get euler angles in yaw, roll, pitch
        float[] targetEulerArray = slerpedQuat.toAngles(new float[3]);
        Matrix3f targetOrientation = rotationMatrix(targetEulerArray[2], targetEulerArray[1], targetEulerArray[0]);

        Vector3f jA = new Vector3f(1, 0, 0);
        Vector3f jB = new Vector3f(0, 1, 0);

        //inverse kinematics stuff
        jA = mechanicalConstInverse_A.mult(targetOrientation).mult(jA);
        jB = mechanicalConstInverse_B.mult(targetOrientation).mult(jB);

        assert jA.y != 0 || jA.z != 0;
        assert jA.y != 0 || jB.z != 0;

        //get roll
        double roll_A = Math.atan2(jA.y, jA.z);
        double roll_B = Math.atan2(jA.y, jA.z);

        //get pitch
        double pitch_A = 2 * Math.acos(jA.x);
        double pitch_B = 2 * Math.acos(jB.x);

        motorAngle_r_A = roll_A;
        motorAngle_p_A = pitch_A;
        motorAngle_r_B = roll_B;
        motorAngle_p_B = pitch_B;

        processC();
    }

    private void processC(){

    }

    private final Matrix4f temp4x4 = new Matrix4f();

    private Matrix3f rotationMatrix(double x, double y, double z) {
        assert temp4x4 != null;
        temp4x4.loadIdentity();
        temp4x4.angleRotation(new Vector3f((float) x, (float) y, (float) z));
        return temp4x4.toRotationMatrix();
    }
}
