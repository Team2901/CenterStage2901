package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.StatesHardware;
import org.firstinspires.ftc.teamcode.OpenCV.ShapeDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.concurrent.TimeUnit;

@Autonomous (name = "States Camera Auto Red Linear Backdrop", group = "Autonomous")
public class StatesRedLinearBackdrop extends LinearOpMode implements OpenCvCamera.AsyncCameraOpenListener {
    StatesHardware robot = new StatesHardware();

    ShapeDetection pipeline = new ShapeDetection(this.telemetry);

    public int spikeMark = 0;
    public int count = 0;
    public OpenCvCamera camera;

    public double xMidInit = 888;
    public boolean movedBack = false;
    public boolean isStopped = false;

    public ElapsedTime cameraTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    //public ElapsedTime preloadTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    public ElapsedTime wristTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public boolean outtakeRightClosed = false;
    public boolean outtakeLeftClosed = false;

    public double outtakeLeftClosedPos = 0.6;
    public double outtakeLeftOpenPos = 0.4;
    public double outtakeRightClosedPos = 0.725;
    public double outtakeRightOpenPos = 0.55;

    public int currentArmTicks = 0;
    double initArmAngle = 60.0;
    double armAngle = initArmAngle;

    public ElapsedTime armTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public enum AutoState {
        CAMERA_WAIT,
        CAMERA_DETECTION,
        MOVE_1,
        MOVE_2,
        MOVE_3,
        STOP
    }

    StatesRedLinearBackdrop.AutoState autoState;

    @Override
    public void runOpMode() throws InterruptedException {
        _init();
        waitForStart();
        robot.rotationServo.setPosition(0.495);
        wristTimer.reset();
        while(wristTimer.time() < 1.5){
            idle();
        }
//        while (opModeIsActive()) {
//            if (isStopped) {
//                break;
//            } else {
//                _loop();
//            }
//        }

        //armTimer.reset();
        //while(armTimer.time(TimeUnit.SECONDS) < 1.25 && opModeIsActive()){
            //robot.lift.setPower(0.1);
            robot.arm.setTargetPosition(2280);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm.setPower(.75);
       // robot.rotationServo.setPosition(.485);
        while(robot.arm.isBusy() && opModeIsActive()){
                idle();
            }
            //robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //0.75 - (((210*robot.arm.getCurrentPosition())/2440) + initArmAngle) - 190) * 0.004
//            if(robot.arm.getCurrentPosition() >= 1410){
//                robot.arm.setTargetPosition(1410);
//                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //robot.arm.setPower(0.56);
                robot.outtakeLeft.setPosition(outtakeLeftOpenPos);
            //}
//            robot.arm.setPower(0);
//            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.arm.setTargetPosition();
        //}
        //robot.lift.setPower(0);

        //robot.outtake.setPosition(0.35);
        //while(armTimer.time(TimeUnit.SECONDS) < 3 && opModeIsActive()){}
        //robot.outtake.setPosition(0.01);
        //while(armTimer.time(TimeUnit.SECONDS) < 4.25 && opModeIsActive()){
          //  robot.lift.setPower(-0.1);
        //}

        //robot.lift.setPower(0);

        while (opModeIsActive()) { idle();}
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

        autoState = StatesRedLinearBackdrop.AutoState.CAMERA_WAIT;

       // preloadTimer.startTime();
        cameraTimer.startTime();
    }

    void _loop() {

        ElapsedTime stall = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        if (count == 0) {
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

        if (autoState == StatesRedLinearBackdrop.AutoState.CAMERA_WAIT) {
            if (cameraTimer.time(TimeUnit.SECONDS) < 7) {
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
                autoState = StatesRedLinearBackdrop.AutoState.CAMERA_DETECTION;
            }
        } else if (autoState == StatesRedLinearBackdrop.AutoState.CAMERA_DETECTION) {
            if (spikeMark == 1) {
                autoState = StatesRedLinearBackdrop.AutoState.MOVE_1;
            } else if (spikeMark == 2) {
                autoState = StatesRedLinearBackdrop.AutoState.MOVE_2;
            } else if (spikeMark == 3) {
                autoState = StatesRedLinearBackdrop.AutoState.MOVE_3;
            }
          //  preloadTimer.reset();
        } else if (autoState == StatesRedLinearBackdrop.AutoState.MOVE_1) {
            if (!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                moveInches(28);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                strafe(-2, 0, 0, 0, 0);
                robot.preload.setPosition(0.8);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                strafe(6, 0, 0, 0, 0);
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
                strafe(12, 0, 0, 0, 0);
                autoState = StatesRedLinearBackdrop.AutoState.STOP;
            }
        } else if (autoState == StatesRedLinearBackdrop.AutoState.MOVE_2) {
            if (!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                moveInches(37);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                //strafe(8, 0, 0, 10, 10);
                //robot.preload.setPosition(0.8);
                robot.outtakeRight.setPosition(outtakeRightOpenPos);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                strafe(6, 0, 0, 0, 0);
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
                autoState = StatesRedLinearBackdrop.AutoState.STOP;
            }
        } else if (autoState == StatesRedLinearBackdrop.AutoState.MOVE_3) {
            if (!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                moveInches(28);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                strafe(22, 0, 0, 30, 30);
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
                strafe(12, 0, 0, 10, 10);
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
                strafe(-10, 0, 0, 0, 0);
                autoState = StatesRedLinearBackdrop.AutoState.STOP;
            }
        } else if (autoState == StatesRedLinearBackdrop.AutoState.STOP) {
            if (!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
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

//        while (opModeIsActive() && robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy()) {
////            telemetry.addData("Current Left Position", robot.frontLeft.getCurrentPosition());
////            telemetry.addData("Current Right Position", robot.frontRight.getCurrentPosition());
//        }

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
