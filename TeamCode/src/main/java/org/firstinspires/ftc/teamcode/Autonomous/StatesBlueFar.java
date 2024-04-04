package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.StatesHardware;
import org.firstinspires.ftc.teamcode.OpenCV.ShapeDetection;
import org.firstinspires.ftc.teamcode.OpenCV.ShapeDetectionBlue;
import org.firstinspires.ftc.teamcode.TeleOp.StatesTeleOp;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.concurrent.TimeUnit;

@Autonomous (name = "States Blue Far", group = "States Autonomous")
public class StatesBlueFar extends LinearOpMode implements OpenCvCamera.AsyncCameraOpenListener {
    StatesHardware robot = new StatesHardware();

    ShapeDetectionBlue pipeline = new ShapeDetectionBlue(this.telemetry);

    public int count = 0;
    public OpenCvCamera camera;

    public double xMidInit = 888;
    public boolean isStopped = false;

    public ElapsedTime cameraTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    public ElapsedTime wristTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    ElapsedTime stall = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public boolean outtakeRightClosed = false;
    public boolean outtakeLeftClosed = false;

    public double outtakeLeftClosedPos = StatesHardware.outtakeLeftClosedPos;
    public double outtakeLeftOpenPos = StatesHardware.outtakeLeftOpenPos;
    public double outtakeRightClosedPos = StatesHardware.outtakeRightClosedPos;
    public double outtakeRightOpenPos = StatesHardware.outtakeRightOpenPos;

    public int currentArmTicks = 0;
    double initArmAngle = 60.0;
    double armAngle = initArmAngle;

    public enum AutoState {
        CAMERA_WAIT,
        CAMERA_DETECTION,
        MOVE_1,
        MOVE_2,
        MOVE_3,
        BACKSTAGE,
        STOP
    }

    StatesBlueFar.AutoState autoState;

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

        autoState = StatesBlueFar.AutoState.CAMERA_WAIT;

//        cameraTimer.startTime();
    }

    void _loop() {
        if (count == 0) {
            cameraTimer.reset();
            count++;
        }

        if (autoState == StatesBlueFar.AutoState.CAMERA_WAIT) {
            if (cameraTimer.time(TimeUnit.SECONDS) < 5) {
                telemetry.addData("Time", cameraTimer.time(TimeUnit.SECONDS));
                telemetry.addData("X Mid", pipeline.xMidVal);
                telemetry.addData("Spike Mark", pipeline.spikeMark);
            } else {
                autoState = StatesBlueFar.AutoState.CAMERA_DETECTION;
            }
        } else if (autoState == StatesBlueFar.AutoState.CAMERA_DETECTION) {
            if (pipeline.spikeMark == 1) {
                autoState = StatesBlueFar.AutoState.MOVE_1;
            } else if (pipeline.spikeMark == 2) {
                autoState = StatesBlueFar.AutoState.MOVE_2;
            } else if (pipeline.spikeMark == 3) {
                autoState = StatesBlueFar.AutoState.MOVE_3;
            }
        } else if (autoState == StatesBlueFar.AutoState.MOVE_1) {
            if (!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                moveInches(31);
                strafe(6,0,0,0,0);
                turnByTicks(-1100); //-90 degrees
                moveInches(6);
                robot.outtakeRight.setPosition(outtakeRightOpenPos);

                stall.reset();
                while(stall.time() < 2){ idle(); }

                moveInches(-9);
                strafe(21.5,0,0,0,0);

                stall.reset();
                while(stall.time() < 1){ idle(); }

                turnByTicks(75);
                moveInches(90);

                stall.reset();
                while(stall.time() < 1){ idle(); }

                autoState = AutoState.BACKSTAGE;
            }
        } else if (autoState == StatesBlueFar.AutoState.MOVE_2) {
            if (!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                moveInches(43);
                turnByTicks(-2100); //-180
                robot.outtakeRight.setPosition(outtakeRightOpenPos);

                stall.reset();
                while(stall.time() < 2){ idle(); }

                moveInches(-3);

                stall.reset();
                while(stall.time() < 1){ idle(); }

                turnByTicks(975); //90 degrees
                moveInches(86);

                stall.reset();
                while(stall.time() < 1){ idle(); }

                autoState = AutoState.BACKSTAGE;
            }
        } else if (autoState == StatesBlueFar.AutoState.MOVE_3) {
            if (!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                moveInches(40);
                strafe(14,0,0,0,0);
                turnByTicks(-2200); //-180 degrees
                robot.outtakeRight.setPosition(outtakeRightOpenPos);

                stall.reset();
                while(stall.time() < 2){ idle(); }

                moveInches(-8);
                turnByTicks(1150); //90 degrees
                moveInches(91);

                stall.reset();
                while(stall.time() < 1){ idle(); }

                autoState = AutoState.BACKSTAGE;
            }
        } else if(autoState == AutoState.BACKSTAGE){
            if (!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
//                turnByTicks(2400); //more than 180
                turnByTicks(-95);
                robot.outtakeLeft.setPosition(outtakeLeftOpenPos);

                stall.reset();
                while(stall.time() < 2){ idle(); }

                robot.arm.setTargetPosition(300);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(0.6);

                autoState = AutoState.STOP;
            }
        }else if (autoState == StatesBlueFar.AutoState.STOP) {
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
