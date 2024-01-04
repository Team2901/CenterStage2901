package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.MecanumDriveHardware;
import org.firstinspires.ftc.teamcode.OpenCV.ShapeDetection;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Shape Detection", group = "Autonomous")
public class MecanumAutoShapes extends OpMode implements OpenCvCamera.AsyncCameraOpenListener{

    MecanumDriveHardware robot = new MecanumDriveHardware();
    ShapeDetection pipeline = new ShapeDetection(this.telemetry);

    public int spikeMark = 1;
    public int count = 0;
    public OpenCvCamera camera;

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
    public void init() {
        robot.init(this.hardwareMap, telemetry);
        ShapeDetection pipeLine;
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        pipeLine = new ShapeDetection(telemetry);
        int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewID);
        camera.setPipeline(pipeLine);
        camera.openCameraDeviceAsync(this);

        autoState = AutoState.CAMERA_WAIT;

        preloadTimer.startTime();
        cameraTimer.startTime();
    }

    @Override
    public void loop() {
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
            if(cameraTimer.time(TimeUnit.SECONDS) < 10) {
                if (pipeline.xMid() < 40) {
                    spikeMark = 1;
                } else if (pipeline.xMid() < 180) {
                    spikeMark = 2;
                } else if (pipeline.xMid() < 320){
                    spikeMark = 3;
                }
                telemetry.addData("X Mid", pipeline.xMid());
                telemetry.addData("Spike Mark", spikeMark);
                telemetry.update();
            }
            autoState = AutoState.CAMERA_DETECTION;
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
                moveInches(20);
                strafe(-3);
                robot.preload.setPosition(0);
                preloadTimer.reset();
                if(preloadTimer.seconds() > 3) {
                    strafe(3);
                    moveInches(-20);
                    strafe(48);
                    autoState = AutoState.STOP;
                }
            }
        } else if(autoState == AutoState.MOVE_2){
            if(!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                moveInches(36);
                strafe(6);
                robot.preload.setPosition(0);
                preloadTimer.reset();
                if(preloadTimer.seconds() > 3) {
                    moveInches(-36);
                    strafe(60);
                }
                autoState = AutoState.STOP;
            }
        } else if(autoState == AutoState.MOVE_3){
            if(!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                moveInches(24);
                strafe(24);
                robot.preload.setPosition(0);
                preloadTimer.reset();
                if(preloadTimer.seconds() > 3) {
                    strafe(-24);
                    moveInches(24);
                    strafe(60);
                }
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

        robot.frontLeft.setPower(0.5);
        robot.frontRight.setPower(0.5);
        robot.backLeft.setPower(0.5);
        robot.backRight.setPower(0.5);

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

    private void strafe(double inches){
        int ticks = (int) (inches * robot.TICKS_PER_INCH);

        robot.frontLeft.setTargetPosition(robot.frontLeft.getCurrentPosition() + ticks);
        robot.frontRight.setTargetPosition(robot.frontRight.getCurrentPosition() - ticks);
        robot.backLeft.setTargetPosition(robot.backLeft.getCurrentPosition() - ticks);
        robot.backRight.setTargetPosition(robot.backRight.getCurrentPosition() + ticks);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft.setPower(0.5);
        robot.frontRight.setPower(0.5);
        robot.backLeft.setPower(0.5);
        robot.backRight.setPower(0.5);

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

    @Override
    public void onOpened() {
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    @Override
    public void onError(int errorCode) {

    }
}