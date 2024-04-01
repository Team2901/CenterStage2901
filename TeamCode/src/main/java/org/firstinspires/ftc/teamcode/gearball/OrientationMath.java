//package org.firstinspires.ftc.teamcode.gearball;
//
//import org.bubblecloud.vecmath.Quaternion;
//import org.bubblecloud.vecmath.Vector3f;
//
//public class OrientationMath {
//    public static Quaternion setQuaternionFromEuler(Quaternion quat, Vector3f euler, String order) {
//        //https://threejs.org/build/three.module.js
//        double x = euler.x;
//        double y = euler.y;
//        double z = euler.z;
//        double c1 = Math.cos(x / 2);
//        double c2 = Math.cos(y / 2);
//        double c3 = Math.cos(z / 2);
//        double s1 = Math.sin(x / 2);
//        double s2 = Math.sin(y / 2);
//        double s3 = Math.sin(z / 2);
//
//        double qx = 0;
//        double qy = 0;
//        double qz = 0;
//        double qw = 0;
//        switch (order) {
//            case "XYZ":
//                qx = s1 * c2 * c3 + c1 * s2 * s3;
//                qy = c1 * s2 * c3 - s1 * c2 * s3;
//                qz = c1 * c2 * s3 + s1 * s2 * c3;
//                qw = c1 * c2 * c3 - s1 * s2 * s3;
//                break;
//            case "YXZ":
//                qx = s1 * c2 * c3 + c1 * s2 * s3;
//                qy = c1 * s2 * c3 - s1 * c2 * s3;
//                qz = c1 * c2 * s3 - s1 * s2 * c3;
//                qw = c1 * c2 * c3 + s1 * s2 * s3;
//                break;
//            case "ZXY":
//                qx = s1 * c2 * c3 - c1 * s2 * s3;
//                qy = c1 * s2 * c3 + s1 * c2 * s3;
//                qz = c1 * c2 * s3 + s1 * s2 * c3;
//                qw = c1 * c2 * c3 - s1 * s2 * s3;
//                break;
//            case "ZYX":
//                qx = s1 * c2 * c3 - c1 * s2 * s3;
//                qy = c1 * s2 * c3 + s1 * c2 * s3;
//                qz = c1 * c2 * s3 - s1 * s2 * c3;
//                qw = c1 * c2 * c3 + s1 * s2 * s3;
//                break;
//            case "YZX":
//                qx = s1 * c2 * c3 + c1 * s2 * s3;
//                qy = c1 * s2 * c3 + s1 * c2 * s3;
//                qz = c1 * c2 * s3 - s1 * s2 * c3;
//                qw = c1 * c2 * c3 - s1 * s2 * s3;
//                break;
//            case "XZY":
//                qx = s1 * c2 * c3 - c1 * s2 * s3;
//                qy = c1 * s2 * c3 - s1 * c2 * s3;
//                qz = c1 * c2 * s3 + s1 * s2 * c3;
//                qw = c1 * c2 * c3 + s1 * s2 * s3;
//                break;
//        }
//        quat.set((float) qx, (float) qy, (float) qz, (float) qw);
//        return quat;
//    }
//
//
//
//}
