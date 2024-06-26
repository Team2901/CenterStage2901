package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.MecanumDriveHardware;
import org.firstinspires.ftc.teamcode.OpenCV.ShapeDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.concurrent.TimeUnit;

@Disabled
@Autonomous(name = "Camera Auto Red Linear Backdrop", group = "Autonomous")
public class RedLinearBackdrop extends LinearOpMode implements OpenCvCamera.AsyncCameraOpenListener{

    MecanumDriveHardware robot = new MecanumDriveHardware();
    ShapeDetection pipeline = new ShapeDetection(this.telemetry);

    public int spikeMark = 0;
    public int count = 0;
    public OpenCvCamera camera;

    public double xMidInit = 888;
    public boolean movedBack = false;
    public boolean isStopped = false;

    public ElapsedTime cameraTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    public ElapsedTime preloadTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    public ElapsedTime liftTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

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
            if(isStopped){
                break;
            } else {
                _loop();
            }
        }

        //slides
        liftTimer.reset();
        while(liftTimer.time(TimeUnit.SECONDS) < 1.25 && opModeIsActive()){
            robot.lift.setPower(0.1);
        }
        robot.lift.setPower(0);
        robot.outtake.setPosition(0.35);
        while(liftTimer.time(TimeUnit.SECONDS) < 3 && opModeIsActive()){}
        robot.outtake.setPosition(0.01);
        while(liftTimer.time(TimeUnit.SECONDS) < 4.25 && opModeIsActive()){
            robot.lift.setPower(-0.1);
        }
        robot.lift.setPower(0);
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
        liftTimer.startTime();
    }


     void _loop() {

        ElapsedTime stall = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        if(count == 0){
            cameraTimer.reset();
            count++;
        }

//        if(pipeline.xMid() > 0) {
//            if (pipeline.xMid() < 10) {
//                spikeMark = 1;
//            }
//            if(pipeline.xMid() < 170) {
//                spikeMark = 2;
//            } else if (pipeline.xMid() < 320) {
//                spikeMark = 3;
//            }
//        }

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
                moveInches(28);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                strafe(-2,0,0,0,0);
                robot.preload.setPosition(0.8);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                strafe(6,0,0,0,0);
                robot.preload.setPosition(0);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                turnByTicks(-645);
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
                strafe(12,0,0,0,0);
                autoState = AutoState.STOP;
            }
        } else if(autoState == AutoState.MOVE_2){
            if(!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                moveInches(37);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                strafe(8,0,0,10,10);
                robot.preload.setPosition(0.8);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                strafe(6,0,0,0,0);
                robot.preload.setPosition(0);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                moveInches(-8.5);

                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                turnByTicks(-630); //-90 degrees + a little bit to account for curving when backing up
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                moveInches(-38);
                autoState = AutoState.STOP;
            }
        } else if(autoState == AutoState.MOVE_3){
            if(!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                moveInches(28);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                strafe(22,0,0,30,30);
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
                robot.preload.setPosition(0);
                strafe(12,0,0,10,10);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                turnByTicks(-645);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                moveInches(-18);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                strafe(-10,0,0,0,0);
                autoState = AutoState.STOP;
            }
        } else if(autoState == AutoState.STOP){
            if(!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);
                isStopped = true;
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
//            robot.preload.setPosition(0);
//            moveInches(-12);
//            strafe(-24);
//        } else if(spikeMark == 2){
//            moveInches(24);
//            robot.preload.setPosition(0);
//            moveInches(-24);
//            strafe(-36);
//            moveInches(24);
//        } else if(spikeMark == 3){
//            moveInches(12);
//            strafe(12);
//            robot.preload.setPosition(0);
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

        while(opModeIsActive() && robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy()){
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