package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.MecanumDriveHardware;
import org.firstinspires.ftc.teamcode.OpenCV.ShapeDetectionBlue;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.concurrent.TimeUnit;

@Disabled
@Autonomous(name = "Camera Auto Blue Linear", group = "Autonomous")
public class MecanumAutoShapesBlueLinear extends LinearOpMode implements OpenCvCamera.AsyncCameraOpenListener{

    MecanumDriveHardware robot = new MecanumDriveHardware();
    ShapeDetectionBlue pipeline = new ShapeDetectionBlue(this.telemetry);

    public int spikeMark = 0;
    public int count = 0;
    public OpenCvCamera camera;

    public double xMidInit = 888;
    public boolean movedBack = false;

    public ElapsedTime cameraTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    public ElapsedTime preloadTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public enum AutoState {
        CAMERA_WAIT,
        CAMERA_DETECTION,
        MOVE_1,
        MOVE_2,
        MOVE_3,
        STOP
    }
    AutoState autoState;

    @Override
    public void runOpMode() throws InterruptedException {
        _init();
        waitForStart();
        while(opModeIsActive()){
            _loop();
        }
    }

    void _init() {
        //why is there a pipeline with capital L? if there's an issue with the vision, this might be the source (i changed it to lowercase)
        //the problem was that i made another ShapeDetection object and it was using that one as the reference
        robot.init(this.hardwareMap, telemetry);
//        ShapeDetection pipeline;
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
//        pipeline = new ShapeDetection(telemetry);
        int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewID);
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(this);

//        xMidInit = pipeline.xMidVal;

        autoState = AutoState.CAMERA_WAIT;

        preloadTimer.startTime();
        cameraTimer.startTime();
    }

    void _loop() {

        if(count == 0){
            cameraTimer.reset();
            count++;
        }

        if(autoState == AutoState.CAMERA_WAIT) {
            if(cameraTimer.time(TimeUnit.SECONDS) < 7) {
                if (pipeline.xMidVal < 130 && pipeline.xMidVal > 5) {
                    spikeMark = 1;
                } else if (pipeline.xMidVal < 280 && pipeline.xMidVal > 5) {
                    spikeMark = 2;
                } else {
                    spikeMark = 3;
                }
                telemetry.addData("Time", cameraTimer.time(TimeUnit.SECONDS));
                telemetry.addData("X Mid", pipeline.xMidVal);
//                telemetry.addData("X Init", xMidInit);
            } else {
                autoState = AutoState.CAMERA_DETECTION;
            }
        } else if(autoState == AutoState.CAMERA_DETECTION){
            if(spikeMark == 1){
                autoState = AutoState.MOVE_1;
            } else if(spikeMark == 2){
                autoState = AutoState.MOVE_2;
            } else if(spikeMark == 3){
                autoState = AutoState.MOVE_3;
            }
            preloadTimer.reset();
        } else if(autoState == AutoState.MOVE_1){
            if(!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                moveInches(26);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                strafe(-12,0,0,0,0);
                try {
                    Thread.sleep(1500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                turnByTicks(620);
                try {
                    Thread.sleep(1500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                moveInches(-10);
                strafe(-3,0,0,0,0);
                robot.preload.setPosition(0.8);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                strafe(3,0,0,0,0);
                robot.preload.setPosition(0.05);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                moveInches(-31);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                strafe(12,0,0,0,0);
                autoState = AutoState.STOP;
            }
        } else if(autoState == AutoState.MOVE_2){
            if(!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                moveInches(31);
                try {
                    Thread.sleep(1500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                strafe(-4,0,0,0,0);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                turnByTicks(615);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                robot.preload.setPosition(0.8);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                strafe(4,0,0,0,0);
                robot.preload.setPosition(0.05);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                moveInches(-42);
                autoState = AutoState.STOP;
            }
        } else if(autoState == AutoState.MOVE_3){
            if(!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                moveInches(28);
                try {
                    Thread.sleep(1500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                strafe(-6,0,0,0,0);
                turnByTicks(580);
                turnByTicks(580);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                moveInches(-6);
                strafe(-7,0,0,0,0);
                robot.preload.setPosition(0.8);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                strafe(7,0,0,0,0);
                robot.preload.setPosition(0.05);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                turnByTicks(-610);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                strafe(12,0,0,40,40);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                moveInches(-38);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                strafe(-12,0,-10,0,0);
                autoState = AutoState.STOP;
            }
        } else if(autoState == AutoState.STOP){
            if(!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);
            }
        }

        telemetry.addData("Spike Mark", spikeMark);
        telemetry.addData("STATE", autoState);
        telemetry.update();
    }

//    @Override
//    public void runOpMode() throws InterruptedException {
//
//
//        while(!opModeIsActive()) {
//            if(pipeline.xMid() > 0) {
//                if (pipeline.xMid() < 107) {
//                    spikeMark = 1;
//                } else if (pipeline.xMid() < 214) {
//                    spikeMark = 2;
//                } else if (pipeline.xMid() < 320) {
//                    spikeMark = 3;
//                }
//            }
//
//            telemetry.addData("X", pipeline.xMid());
//            telemetry.addData("Spike Mark", spikeMark);
//            telemetry.update();
//        }
//
//        waitForStart();
//
//        //move forward, strafe (if 1 or 2), deposit purple, move back, strafe left to park, deposit yellow (for BLUE side)
////        timer.reset();
//        if(spikeMark == 1){
//            moveInches(12);
//            strafe(-12);
//            robot.preload.setPosition(0.05;
//            moveInches(-12);
//            strafe(-24);
//        } else if(spikeMark == 2){
//            moveInches(24);
//            robot.preload.setPosition(0.05;
//            moveInches(-24);
//            strafe(-36);
//            moveInches(24);
//        } else if(spikeMark == 3){
//            moveInches(12);
//            strafe(12);
//            robot.preload.setPosition(0.05;
//            moveInches(-12);
//            strafe(-48);
//            moveInches(48);
//        }
////        robot.intake.setPower(-0.8);
//    }

    private void moveInches(double inches){
        int ticks = (int) (inches * robot.TICKS_PER_INCH);

        robot.frontLeft.setTargetPosition(robot.frontLeft.getCurrentPosition() + ticks);
        robot.frontRight.setTargetPosition(robot.frontRight.getCurrentPosition() + ticks);
        robot.backLeft.setTargetPosition(robot.backLeft.getCurrentPosition() + ticks);
        robot.backRight.setTargetPosition(robot.backRight.getCurrentPosition() + ticks);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft.setPower(0.35);
        robot.frontRight.setPower(0.35);
        robot.backLeft.setPower(0.35);
        robot.backRight.setPower(0.35);

        while(robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy()){
//            telemetry.addData("Current Left Position", robot.backLeft.getCurrentPosition());
//            telemetry.addData("Current Right Position", robot.backRight.getCurrentPosition());
        }

        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void strafe(double inches, int FLTicks, int FRTicks, int BLTicks, int BRTicks){
        int ticks = (int) (inches * robot.TICKS_PER_INCH);

        robot.frontLeft.setTargetPosition(robot.frontLeft.getCurrentPosition() - ticks - FLTicks);
        robot.frontRight.setTargetPosition(robot.frontRight.getCurrentPosition() + ticks + FRTicks);
        robot.backLeft.setTargetPosition(robot.backLeft.getCurrentPosition() + ticks + BLTicks);
        robot.backRight.setTargetPosition(robot.backRight.getCurrentPosition() - ticks - BRTicks);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft.setPower(0.35);
        robot.frontRight.setPower(0.35);
        robot.backLeft.setPower(0.35);
        robot.backRight.setPower(0.35);

        while(robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy()){
//            telemetry.addData("Current Left Position", robot.frontLeft.getCurrentPosition());
//            telemetry.addData("Current Right Position", robot.frontRight.getCurrentPosition());
        }

        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnByTicks(int ticks){
        int FLTicks = ticks;
        int FRTicks = -ticks;
        int BLTicks = ticks;
        int BRTicks = -ticks;

        robot.frontLeft.setTargetPosition(robot.frontLeft.getCurrentPosition() + FLTicks);
        robot.frontRight.setTargetPosition(robot.frontRight.getCurrentPosition() + FRTicks);
        robot.backLeft.setTargetPosition(robot.backLeft.getCurrentPosition() + BLTicks);
        robot.backRight.setTargetPosition(robot.backRight.getCurrentPosition() + BRTicks);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft.setPower(0.35);
        robot.frontRight.setPower(0.35);
        robot.backLeft.setPower(0.35);
        robot.backRight.setPower(0.35);

        while(opModeIsActive() && robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy()){
//            telemetry.addData("Current Left Position", robot.frontLeft.getCurrentPosition());
//            telemetry.addData("Current Right Position", robot.frontRight.getCurrentPosition());
        }

        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void onOpened() {
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    @Override
    public void onError(int errorCode) {

    }
}