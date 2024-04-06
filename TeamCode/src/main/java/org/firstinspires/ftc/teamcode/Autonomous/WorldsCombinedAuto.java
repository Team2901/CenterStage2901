package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.StatesHardware;
import org.firstinspires.ftc.teamcode.Hardware.StatesHardware.*;
import org.firstinspires.ftc.teamcode.OpenCV.ShapeDetection;
import org.firstinspires.ftc.teamcode.TeleOp.StatesTeleOp;
import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.concurrent.TimeUnit;

@Autonomous (name = "Worlds Combined Auto", group = "AAutonomous")
public class WorldsCombinedAuto extends LinearOpMode implements OpenCvCamera.AsyncCameraOpenListener {
    StatesHardware robot = new StatesHardware();
    ImprovedGamepad impGamepad1;
    ImprovedGamepad impGamepad2;

    ShapeDetection pipeline = new ShapeDetection(this.telemetry, robot);

    public static final int IMGPROC_SAMPLES = 10;

    public int count = 0;
    public OpenCvCamera camera;
    public int spikeMark;

    public boolean isStopped = false;

    public ElapsedTime cameraTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    public ElapsedTime wristTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    ElapsedTime stall = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public double outtakeLeftOpenPos = StatesHardware.outtakeLeftOpenPos;
    public double outtakeRightOpenPos = StatesHardware.outtakeRightOpenPos;

    double waitSec = 0;

    public enum AutoState {
        STALL,
        CAMERA_WAIT,
        CAMERA_DETECTION,
        MOVE_1,
        MOVE_2,
        MOVE_3,
        BACKDROP,
        BACKSTAGE,
        STOP
    }
    AutoState autoState;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        _init();

        while(opModeInInit()){
            impGamepad1.update();

            telemetry.addLine("Red (B) or Blue (X)?");
            if(impGamepad1.x.isInitialPress()){
                robot.alliance = Alliance.BLUE;
            } else if(impGamepad1.b.isInitialPress()){
                robot.alliance = Alliance.RED;
            }
            telemetry.addData("Alliance", robot.alliance);

            telemetry.addLine("Far (Y) or Close (A)?");
            if(impGamepad1.y.isInitialPress()){
                robot.startLocation = StartLocation.FAR;
            } else if(impGamepad1.a.isInitialPress()){
                robot.startLocation = StartLocation.CLOSE;
            }
            telemetry.addData("Starting Location", robot.startLocation);

            telemetry.addLine("Add wait time? (D-pad Up or Down)");
            if(impGamepad1.dpad_up.isInitialPress()){
                waitSec += 0.5;
            } else if(impGamepad1.dpad_down.isInitialPress() && waitSec >= 0.5){
                waitSec -= 0.5;
            }
            telemetry.addData("Stall Time", waitSec);

            telemetry.addLine("Park in the Middle (D-pad Left) or Corner (D-pad Right)?");
            if(impGamepad1.dpad_left.isInitialPress()){
                robot.parkLocation = ParkLocation.MIDDLE;
            } else if(impGamepad1.dpad_right.isInitialPress()){
                robot.parkLocation = ParkLocation.CORNER;
            }
            telemetry.addData("Parking Location", robot.parkLocation);

            telemetry.update();
        }

        waitForStart();
        moveArm(150);
        stall(0.1);
        robot.rotationServo.setPosition(0.1);

        while (opModeIsActive()) {
            if (isStopped) {
                break;
            } else {
                _loop();
            }
        }

        while (opModeIsActive()) { idle();}
    }

    void _init() {
        impGamepad1 = new ImprovedGamepad(gamepad1, new ElapsedTime(), "gamepad1");
        impGamepad2 = new ImprovedGamepad(gamepad1, new ElapsedTime(), "gamepad2");
        robot.init(this.hardwareMap, telemetry);

        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewID);
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(this);
        FtcDashboard.getInstance().startCameraStream(camera,0);

        robot.imu.resetYaw();
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        autoState = AutoState.STALL;
    }

    //base code from StatesRedClose
    void _loop() {
        if (count == 0) {
            cameraTimer.reset();
            count++;
        }

        if(autoState == AutoState.STALL){
            stall(waitSec);
            autoState = AutoState.CAMERA_WAIT;
        } else if (autoState == AutoState.CAMERA_WAIT) {
//            if (cameraTimer.time(TimeUnit.SECONDS) < 5) {
//                telemetry.addData("Time", cameraTimer.time(TimeUnit.SECONDS));
//                telemetry.addData("X Mid", pipeline.xMidVal);
//                telemetry.addData("Spike Mark", pipeline.spikeMark);
//            } else {
//                autoState = AutoState.CAMERA_DETECTION;
//            }
            long startTime = System.currentTimeMillis();

            //Wait until # of frames processed
            while(camera.getFrameCount() < IMGPROC_SAMPLES){}
            telemetry.addData("Image processing time", (System.currentTimeMillis() - startTime)/1000.0);
            telemetry.addData("X Mid", pipeline.xMidVal);
            telemetry.addData("Spike Mark", pipeline.spikeMark);
            autoState = AutoState.CAMERA_DETECTION;
        } else if (autoState == AutoState.CAMERA_DETECTION) {
            if (pipeline.spikeMark == 1) {
                autoState = AutoState.MOVE_1;
            } else if (pipeline.spikeMark == 2) {
                autoState = AutoState.MOVE_2;
            } else if (pipeline.spikeMark == 3) {
                autoState = AutoState.MOVE_3;
            }

            spikeMark = pipeline.spikeMark;
        } else if (autoState == AutoState.MOVE_1) {
            if(robot.alliance == Alliance.RED && robot.startLocation == StartLocation.CLOSE) {
                moveInches(28);
                strafe(3, 0, 0, 0, 0);
                turnByTicks(-1100); //-90 degrees
                moveInches(2);
                robot.outtakeRight.setPosition(outtakeRightOpenPos);

                stall(2);

                moveInches(-32);

                stall(2);

                strafe(3, 0, 0, 0, 0);
                autoState = AutoState.BACKDROP;
            } else if(robot.alliance == Alliance.BLUE && robot.startLocation == StartLocation.CLOSE){
                moveInches(30);
                strafe(-24, 0, 0, 0, 0);
                turnByTicks(1070); //90 degrees - a little
                robot.outtakeRight.setPosition(outtakeRightOpenPos);

                stall(2);

                moveInches(-11);

                stall(2);

                strafe(21, 0, 0, 0, 0);
                autoState = AutoState.BACKDROP;
            } else if(robot.alliance == Alliance.RED && robot.startLocation == StartLocation.FAR){
                if(robot.parkLocation == ParkLocation.MIDDLE){
                    moveInches(40);
                    strafe(-12, 0, 0, 0, 0);
                    turnByTicks(-2200); //-180 degrees

                    robot.outtakeRight.setPosition(outtakeRightOpenPos);
                    stall(2);

                    moveInches(-8.5);
                    turnByTicks(-850);
                } else {
                    moveInches(22);
                    strafe(-12,0,0,0,0);
                    robot.outtakeRight.setPosition(outtakeRightOpenPos);
                    stall(2);
                    moveInches(-21);
                    turnByTicks(1100);
                }

                moveInches(91);
                stall(2);

                autoState = AutoState.BACKSTAGE;
            } else { //blue far
                moveInches(31);
                strafe(6, 0, 0, 0, 0);
                turnByTicks(-1100); //-90 degrees
                moveInches(6);
                robot.outtakeRight.setPosition(outtakeRightOpenPos);

                stall(2);

                moveInches(-9);

                if(robot.parkLocation == ParkLocation.MIDDLE) {
                    strafe(21.5, 0, 0, 0, 0);

                    stall(1);

                    turnByTicks(75);
                } else {
                    strafe(-30,0,0,0,0);
                }

                moveInches(90);
                stall(1);

                autoState = AutoState.BACKSTAGE;
            }
        } else if (autoState == AutoState.MOVE_2) {
            if (robot.alliance == Alliance.RED && robot.startLocation == StartLocation.CLOSE) {
                moveInches(27);
                robot.outtakeRight.setPosition(outtakeRightOpenPos);

                stall(2);

                moveInches(-4);
                turnByTicks(-1100); //-90 degrees
                moveInches(-32);

                stall(2);

                strafe(3,0,0,0,0);
                turnByTicks(-30);

                autoState = AutoState.BACKDROP;
            } else if(robot.alliance == Alliance.BLUE && robot.startLocation == StartLocation.CLOSE){
                moveInches(27);
                robot.outtakeRight.setPosition(outtakeRightOpenPos);

                stall(2);

                moveInches(-4);
                turnByTicks(1070); //90 degrees
                moveInches(-32.5);

                stall(2);

                strafe(3,0,0,0,0);
                turnByTicks(30);

                autoState = AutoState.BACKDROP;
            } else if(robot.alliance == Alliance.RED && robot.startLocation == StartLocation.FAR){
                if(robot.parkLocation == ParkLocation.MIDDLE) {
                    moveInches(43);
                    turnByTicks(-2100); //-180

                    robot.outtakeRight.setPosition(outtakeRightOpenPos);
                    stall(2);

                    moveInches(-5);
                    stall(1);
                    turnByTicks(-850);
                } else {
                    moveInches(25);

                    robot.outtakeRight.setPosition(outtakeRightOpenPos);
                    stall(2);

                    moveInches(-24);
                    turnByTicks(1100);
                }

                moveInches(86);
                stall(2);

                autoState = AutoState.BACKSTAGE;
            } else { //blue far
                if(robot.parkLocation == ParkLocation.MIDDLE) {
                    moveInches(43);
                    turnByTicks(-2100); //-180

                    robot.outtakeRight.setPosition(outtakeRightOpenPos);
                    stall(2);

                    moveInches(-3);
                    stall(1);
                    turnByTicks(975); //90 degrees
                } else {
                    moveInches(25);

                    robot.outtakeRight.setPosition(outtakeRightOpenPos);
                    stall(2);

                    moveInches(-24);
                    turnByTicks(-850);
                }

                moveInches(86);

                stall(1);

                autoState = AutoState.BACKSTAGE;
            }
        } else if (autoState == AutoState.MOVE_3) {
            if (robot.alliance == Alliance.RED && robot.startLocation == StartLocation.CLOSE) {
                moveInches(25);
                strafe(24, 0, 0, 0, 0);
                turnByTicks(-1100); //-90 degrees
                robot.outtakeRight.setPosition(outtakeRightOpenPos);

                stall(2);

                moveInches(-11);

                stall(2);

                strafe(-14, 0, 0, 0, 0);
                autoState = AutoState.BACKDROP;
            } else if(robot.alliance == Alliance.BLUE && robot.startLocation == StartLocation.CLOSE){
                moveInches(31);
                strafe(-3,0,0,0,0);
                turnByTicks(1100); //90 degrees
                moveInches(3);
                robot.outtakeRight.setPosition(outtakeRightOpenPos);

                stall(2);

                moveInches(-32.5);

                stall(1);

                strafe(3, 0, 0, 0, 0);
                turnByTicks(-80);
                autoState = AutoState.BACKDROP;
            } else if(robot.alliance == Alliance.RED && robot.startLocation == StartLocation.FAR){
                moveInches(31);
                strafe(-6, 0, 0, 0, 0);
                turnByTicks(1100); //90 degrees
                moveInches(6);
                robot.outtakeRight.setPosition(outtakeRightOpenPos);

                stall(2);

                moveInches(-9);

                if(robot.parkLocation == ParkLocation.MIDDLE) {
                    strafe(-22, 0, 0, 0, 0);

                    stall(1);

//                turnByTicks(-2050); //-180 degrees (idk why this one takes a lot less ticks than the other 180's???)
                    turnByTicks(-75);
                } else {
                    strafe(30,0,0,0,0);
                }

                moveInches(90);

                stall(2);

                autoState = AutoState.BACKSTAGE;
            } else { //blue far
                if(robot.parkLocation == ParkLocation.MIDDLE) {
                    moveInches(40);
                    strafe(14, 0, 0, 0, 0);
                    turnByTicks(-2200); //-180 degrees
                    robot.outtakeRight.setPosition(outtakeRightOpenPos);

                    stall(2);

                    moveInches(-8);
                    turnByTicks(1150); //90 degrees
                } else {
                    moveInches(22);
                    strafe(14,0,0,0,0);

                    robot.outtakeRight.setPosition(outtakeRightOpenPos);
                    stall(2);

                    moveInches(-21);
                    turnByTicks(-850);
                }

                moveInches(91);

                stall(1);

                autoState = AutoState.BACKSTAGE;
            }
        } else if(autoState == AutoState.BACKDROP){
                robot.rotationServo.setPosition(0.495);

                wristTimer.reset();
                while (wristTimer.time() < 1.5) { idle(); }

                moveArm(robot.maxHeightArmTicks);

                while (robot.arm.isBusy() && opModeIsActive()) { idle(); }

                robot.outtakeLeft.setPosition(outtakeLeftOpenPos);

                stall(3);

                moveArm(150);

                robot.rotationServo.setPosition(0.1);

                if(robot.parkLocation == ParkLocation.CORNER) {
                    if (robot.alliance == Alliance.RED) {
                        if (spikeMark == 1) {
                            strafe(-38, 0, 0, 0, 0);
                        } else if (spikeMark == 2) {
                            strafe(-30, 0, 0, 0, 0);
                        } else if (spikeMark == 3) {
                            strafe(-20, 0, 0, 0, 0);
                        }
                    } else if (robot.alliance == Alliance.BLUE) {
                        if (spikeMark == 1) {
                            strafe(18, 0, 0, 0, 0);
                        } else if (spikeMark == 2) {
                            strafe(28, 0, 0, 0, 0);
                        } else if (spikeMark == 3) {
                            strafe(38, 0, 0, 0, 0);
                        }
                    }
                }

                if(robot.parkLocation == ParkLocation.MIDDLE){
                    if(robot.alliance == Alliance.RED){
                        if(spikeMark == 1){
                            strafe(18,0,0,0,0);
                        } else if(spikeMark == 2){
                            strafe(28,0,0,0,0);
                        } else if(spikeMark == 3){
                            strafe(38,0,0,0,0);
                        }
                    } else if(robot.alliance == Alliance.BLUE){ // has not been tested yet
                        if (spikeMark == 1) {
                            strafe(-38, 0, 0, 0, 0);
                        } else if (spikeMark == 2) {
                            strafe(-30, 0, 0, 0, 0);
                        } else if (spikeMark == 3) {
                            strafe(-20, 0, 0, 0, 0);
                        }
                    }
                }

                autoState = AutoState.STOP;

        }else if(autoState == AutoState.BACKSTAGE){
                if(robot.alliance == Alliance.RED){
                    turnByTicks(95);
                } else if(robot.alliance == Alliance.BLUE){
                    turnByTicks(-95);
                }

                robot.outtakeLeft.setPosition(outtakeLeftOpenPos);

                stall(2);

                robot.arm.setTargetPosition(300);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(0.6);

                autoState = AutoState.STOP;
        }else if (autoState == AutoState.STOP) {
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);
                isStopped = true;
        }

        telemetry.addData("Spike Mark", pipeline.spikeMark);
        telemetry.addData("STATE", autoState);
        telemetry.update();
    }

    private void moveInches(double inches) {
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

        while (opModeIsActive() && robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy()) {
//            telemetry.addData("Current Left Position", robot.backLeft.getCurrentPosition());
//            telemetry.addData("Current Right
//            public void onError ( int errorCode){
//
//            }
        }
    }

    private void strafe(double inches, int FLTicks, int FRTicks, int BLTicks, int BRTicks) {
        int ticks = (int) (inches * robot.TICKS_PER_INCH);

        robot.frontLeft.setTargetPosition(robot.frontLeft.getCurrentPosition() + ticks + FLTicks);
        robot.frontRight.setTargetPosition(robot.frontRight.getCurrentPosition() - ticks - FRTicks);
        robot.backLeft.setTargetPosition(robot.backLeft.getCurrentPosition() - ticks - BLTicks);
        robot.backRight.setTargetPosition(robot.backRight.getCurrentPosition() + ticks + BRTicks);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft.setPower(0.35);
        robot.frontRight.setPower(0.35);
        robot.backLeft.setPower(0.35);
        robot.backRight.setPower(0.35);

        while (opModeIsActive() && robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy()) {
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

    public void turnByTicks(int ticks) {
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

        while (opModeIsActive() && robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy()) {
//            telemetry.addData("Current Left Position", robot.frontLeft.getCurrentPosition());
//            telemetry.addData("Current Right Position", robot.frontRight.getCurrentPosition());
        }
    }

    public void moveArm(int ticks){
        robot.arm.setTargetPosition(ticks);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(0.65);
    }

    public void stall(double seconds){
        stall.reset();
        while(stall.time() < seconds){ idle(); }
    }

    @Override
    public void onOpened() {
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    @Override
    public void onError(int errorCode) {

    }
}