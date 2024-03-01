package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.OpenCV.ShapeDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public abstract class AutoOpMode extends LinearOpMode implements OpenCvCamera.AsyncCameraOpenListener {
    public Hardware robot;
    public AutoState autoState;
    public OpenCvCamera camera;
    public ShapeDetection pipeline;

    public ElapsedTime cameraTimer;
    public ElapsedTime wristTimer;
    public ElapsedTime stall;

    public boolean isStopped = false;

    public void runOpMode() throws InterruptedException {
        _init();
        waitForStart();

        robot.arm.setTargetPosition(150);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(0.1);


        stall = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        while (stall.time() < 0.1) {
            idle();
        }

        robot.rotationServo.setPosition(0.1);

        while (opModeIsActive()) {
            if (isStopped) {
                break;
            } else {
                _loop();
            }
        }

        while (opModeIsActive()) {
            idle();
        }

        stall.reset();
        cameraTimer  = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        wristTimer  = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    }

    void _init() {
        robot = new Hardware();
        robot.init(this.hardwareMap, telemetry);
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewID);
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(this);

        autoState = AutoState.CAMERA_WAIT;

    }

    abstract void _loop();

    public void moveInches(double inches) {
        int ticks = (int) (inches * robot.TICKS_PER_INCH);

        robot.frontLeft.setTargetPosition(robot.frontLeft.getCurrentPosition() + ticks);
        robot.frontRight.setTargetPosition(robot.frontRight.getCurrentPosition() + ticks);
        robot.backLeft.setTargetPosition(robot.backLeft.getCurrentPosition() + ticks);
        robot.backRight.setTargetPosition(robot.backRight.getCurrentPosition() + ticks);

        robot.setWheelRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setWheelPower(0.35);

        while (opModeIsActive() && robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy()) {
//            telemetry.addData("Current Left Position", robot.backLeft.getCurrentPosition());
//            telemetry.addData("Current Right
//            public void onError ( int errorCode){
//
//            }
        }
    }

    public void strafe(double inches, int FLTicks, int FRTicks, int BLTicks, int BRTicks) {
        int ticks = (int) (inches * robot.TICKS_PER_INCH);

        robot.frontLeft.setTargetPosition(robot.frontLeft.getCurrentPosition() + ticks + FLTicks);
        robot.frontRight.setTargetPosition(robot.frontRight.getCurrentPosition() - ticks - FRTicks);
        robot.backLeft.setTargetPosition(robot.backLeft.getCurrentPosition() - ticks - BLTicks);
        robot.backRight.setTargetPosition(robot.backRight.getCurrentPosition() + ticks + BRTicks);

        robot.setWheelRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setWheelPower(0.35);

        while (opModeIsActive() && robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy()) {
////            telemetry.addData("Current Left Position", robot.frontLeft.getCurrentPosition());
////            telemetry.addData("Current Right Position", robot.frontRight.getCurrentPosition());
        }

        robot.setWheelPower(0);


        robot.setWheelRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnByTicks(int ticks) {
        int FLTicks = ticks;
        int FRTicks = -ticks;
        int BLTicks = ticks;
        int BRTicks = -ticks;

        robot.frontLeft.setTargetPosition(robot.frontLeft.getCurrentPosition() + FLTicks);
        robot.frontRight.setTargetPosition(robot.frontRight.getCurrentPosition() + FRTicks);
        robot.backLeft.setTargetPosition(robot.backLeft.getCurrentPosition() + BLTicks);
        robot.backRight.setTargetPosition(robot.backRight.getCurrentPosition() + BRTicks);

        robot.setWheelRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setWheelPower(0.35);

        while (opModeIsActive() && robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy()) {
//            telemetry.addData("Current Left Position", robot.frontLeft.getCurrentPosition());
//            telemetry.addData("Current Right Position", robot.frontRight.getCurrentPosition());
        }
    }

    public void onOpened() {
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    public void onError(int errorCode) {

    }
}
