package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.MecanumDriveHardware;
import org.firstinspires.ftc.teamcode.Vision.ShapeDetectionBlue;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Shape Detection Blue", group = "Autonomous")
public class MecanumAutoShapesBlue extends OpMode implements OpenCvCamera.AsyncCameraOpenListener{

    MecanumDriveHardware robot = new MecanumDriveHardware();
    ShapeDetectionBlue pipeline = new ShapeDetectionBlue(this.telemetry);

//    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public int spikeMark = 0;
    public int count = 0;
    public OpenCvCamera camera;

    @Override
    public void init() {
        robot.init(this.hardwareMap, telemetry);
        ShapeDetectionBlue pipeLine;
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        pipeLine = new ShapeDetectionBlue(telemetry);
        int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewID);
        camera.setPipeline(pipeLine);
        camera.openCameraDeviceAsync(this);
    }

    @Override
    public void loop() {
        if(pipeline.xCoord() > 0) {
            if (pipeline.xCoord() < 107) {
                spikeMark = 1;
            } else if (pipeline.xCoord() < 214) {
                spikeMark = 2;
            } else if (pipeline.xCoord() < 320) {
                spikeMark = 3;
            }
        }

        telemetry.addData("X", pipeline.xCoord());
        telemetry.addData("Spike Mark", spikeMark);
        telemetry.update();

        if(count == 0) {
            if (spikeMark == 1) {
                moveInches(12);
                strafe(-12);
                robot.preload.setPosition(0);
                moveInches(-12);
                strafe(-24);
            } else if (spikeMark == 2) {
                moveInches(24);
                robot.preload.setPosition(0);
                moveInches(-24);
                strafe(-36);
                moveInches(24);
            } else if (spikeMark == 3) {
                moveInches(12);
                strafe(12);
                robot.preload.setPosition(0);
                moveInches(-12);
                strafe(-48);
                moveInches(48);
            }
        }

        count++;
    }

//    @Override
//    public void runOpMode() throws InterruptedException {
//
//
//        while(!opModeIsActive()) {
//            if(pipeline.xCoord() > 0) {
//                if (pipeline.xCoord() < 107) {
//                    spikeMark = 1;
//                } else if (pipeline.xCoord() < 214) {
//                    spikeMark = 2;
//                } else if (pipeline.xCoord() < 320) {
//                    spikeMark = 3;
//                }
//            }
//
//            telemetry.addData("X", pipeline.xCoord());
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
            telemetry.addData("Current Left Position", robot.frontLeft.getCurrentPosition());
            telemetry.addData("Current Right Position", robot.frontRight.getCurrentPosition());
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
            telemetry.addData("Current Left Position", robot.frontLeft.getCurrentPosition());
            telemetry.addData("Current Right Position", robot.frontRight.getCurrentPosition());
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
        camera.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);
    }

    @Override
    public void onError(int errorCode) {

    }
}