package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.StatesHardware;
import org.firstinspires.ftc.teamcode.OpenCV.ShapeDetection;
import org.firstinspires.ftc.teamcode.TeleOp.StatesTeleOp;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.concurrent.TimeUnit;

@Autonomous (name = "States Red Close", group = "States Autonomous")
public class StatesRedClose extends LinearOpMode implements OpenCvCamera.AsyncCameraOpenListener {
    StatesHardware robot = new StatesHardware();

    ShapeDetection pipeline = new ShapeDetection(this.telemetry);

    public int count = 0;
    public OpenCvCamera camera;
    public int spikeMark;

    public double xMidInit = 888;
    public boolean isStopped = false;

    public ElapsedTime cameraTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    public ElapsedTime wristTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    ElapsedTime stall = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public boolean outtakeRightClosed = false;
    public boolean outtakeLeftClosed = false;

    public double outtakeLeftClosedPos = StatesTeleOp.outtakeLeftClosedPos;
    public double outtakeLeftOpenPos = StatesTeleOp.outtakeLeftClosedPos - 0.2;
    public double outtakeRightClosedPos = StatesTeleOp.outtakeRightClosedPos;
    public double outtakeRightOpenPos = StatesTeleOp.outtakeRightClosedPos - 0.2;

    public int currentArmTicks = 0;
    double initArmAngle = 60.0;
    double armAngle = initArmAngle;

    public enum AutoState {
        CAMERA_WAIT,
        CAMERA_DETECTION,
        MOVE_1,
        MOVE_2,
        MOVE_3,
        BACKDROP,
        STOP
    }

    StatesRedClose.AutoState autoState;

    @Override
    public void runOpMode() throws InterruptedException {
        _init();
        waitForStart();

        robot.arm.setTargetPosition(150);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(0.1);

        stall.reset();
        while(stall.time() < 0.1){ idle(); }

        robot.rotationServo.setPosition(0.1);

        while (opModeIsActive()) {
            if (isStopped) {
                break;
            } else {
                _loop();
            }
        }

//            robot.arm.setPower(0);
//            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.arm.setTargetPosition();

        while (opModeIsActive()) { idle();}
    }

    void _init() {
        robot.init(this.hardwareMap, telemetry);
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewID);
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(this);

        autoState = StatesRedClose.AutoState.CAMERA_WAIT;

//        cameraTimer.startTime();
    }

    void _loop() {
        if (count == 0) {
            cameraTimer.reset();
            count++;
        }

        if (autoState == StatesRedClose.AutoState.CAMERA_WAIT) {
            if (cameraTimer.time(TimeUnit.SECONDS) < 5) {
                telemetry.addData("Time", cameraTimer.time(TimeUnit.SECONDS));
                telemetry.addData("X Mid", pipeline.pixelXValAverage);
                telemetry.addData("Spike Mark", pipeline.spikeMark);
            } else {
                autoState = StatesRedClose.AutoState.CAMERA_DETECTION;
            }
        } else if (autoState == StatesRedClose.AutoState.CAMERA_DETECTION) {
            if (pipeline.spikeMark == 1) {
                autoState = StatesRedClose.AutoState.MOVE_1;
            } else if (pipeline.spikeMark == 2) {
                autoState = StatesRedClose.AutoState.MOVE_2;
            } else if (pipeline.spikeMark == 3) {
                autoState = StatesRedClose.AutoState.MOVE_3;
            }

            spikeMark = pipeline.spikeMark;
        } else if (autoState == StatesRedClose.AutoState.MOVE_1) {
            if (!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                moveInches(28);
                strafe(3,0,0,0,0);
                turnByTicks(-1100); //-90 degrees
                moveInches(2);
                robot.outtakeRight.setPosition(outtakeRightOpenPos);

                stall.reset();
                while(stall.time() < 2){ idle(); }

                moveInches(-32);

                stall.reset();
                while(stall.time() < 2){ idle(); }

                strafe(3, 0, 0, 0, 0);
                autoState = AutoState.BACKDROP;
            }
        } else if (autoState == StatesRedClose.AutoState.MOVE_2) {
            if (!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                moveInches(27);
                robot.outtakeRight.setPosition(outtakeRightOpenPos);

                stall.reset();
                while(stall.time() < 2){ idle(); }

                moveInches(-4);
                turnByTicks(-1100); //-90 degrees
                moveInches(-32);

                stall.reset();
                while(stall.time() < 2){ idle(); }

                strafe(3,0,0,0,0);
                turnByTicks(-30);

                autoState = AutoState.BACKDROP;
            }
        } else if (autoState == StatesRedClose.AutoState.MOVE_3) {
            if (!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                moveInches(25);
                strafe(24, 0, 0, 0, 0);
                turnByTicks(-1100); //-90 degrees
                robot.outtakeRight.setPosition(outtakeRightOpenPos);

                stall.reset();
                while(stall.time() < 2){ idle(); }

                moveInches(-11);

                stall.reset();
                while(stall.time() < 2){ idle(); }

                strafe(-14, 0, 0, 0, 0);
                autoState = AutoState.BACKDROP;
            }
        } else if(autoState == AutoState.BACKDROP){
            if (!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                robot.rotationServo.setPosition(0.495);

                wristTimer.reset();
                while (wristTimer.time() < 1.5) { idle(); }

                robot.arm.setTargetPosition(2280);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(0.65);

                while (robot.arm.isBusy() && opModeIsActive()) { idle(); }

                robot.outtakeLeft.setPosition(outtakeLeftOpenPos);

                stall.reset();
                while(stall.time() < 3){ idle(); }

                robot.arm.setTargetPosition(150);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(0.65);

                robot.rotationServo.setPosition(0.1);

                //to move into the corner
                if(spikeMark == 1){
                    strafe(-38,0,0,0,0);
                } else if(spikeMark == 2){
                    strafe(-30,0,0,0,0);
                } else if(spikeMark == 3){
                    strafe(-20,0,0,0,0);
                }

                //move to middle
//                if(spikeMark == 1){
//                    strafe(18,0,0,0,0);
//                } else if(spikeMark == 2){
//                    strafe(28,0,0,0,0);
//                } else if(spikeMark == 3){
//                    strafe(38,0,0,0,0);
//                }

                autoState = AutoState.STOP;
            }
        }else if (autoState == StatesRedClose.AutoState.STOP) {
            if (!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                robot.frontLeft.setPower(0);
                robot.frontRight.setPower(0);
                robot.backLeft.setPower(0);
                robot.backRight.setPower(0);
                isStopped = true;
            }
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
////            telemetry.addData("Current Left Position", robot.frontLeft.getCurrentPosition());
////            telemetry.addData("Current Right Position", robot.frontRight.getCurrentPosition());
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
    @Override
    public void onOpened() {
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    @Override
    public void onError(int errorCode) {

    }
}
