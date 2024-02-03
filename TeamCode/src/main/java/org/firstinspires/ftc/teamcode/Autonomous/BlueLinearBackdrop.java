package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "Camera Auto Blue Linear Backdrop", group = "Autonomous")
public class BlueLinearBackdrop extends LinearOpMode implements OpenCvCamera.AsyncCameraOpenListener{

    MecanumDriveHardware robot = new MecanumDriveHardware();
    ShapeDetectionBlue pipeline = new ShapeDetectionBlue(this.telemetry);

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

        // Code Review: This should be done inside of _loop. Or at least inside of the while loop above.
        //              Or really move all of _loop() into the while loop above.
        //              Also, it should be incorporated into the state machine.
        // Code Review: Now you have time to use encoders instead of time for the backdrop. Also,
        //              the mechanism will be changed to an arm.
        //slides
        liftTimer.reset();
        while(liftTimer.time(TimeUnit.SECONDS) < 1.5 && opModeIsActive()){
            robot.lift.setPower(0.3); // Code Review: Now you have time to use encoders instead.
        }
        robot.lift.setPower(0);
        robot.outtake.setPosition(0.35);
        while(liftTimer.time(TimeUnit.SECONDS) < 3 && opModeIsActive()) {
            idle();
        }
        // Code review: Note that in the lines below, the lift is going down while the servo is also
        //              moving back. Did you intend to wait for the servo to move back before moving
        //              the lift down? Do you even need to move the lift back down, or move the
        //              servo back?
        robot.outtake.setPosition(0.01);
        while(liftTimer.time(TimeUnit.SECONDS) < 4.5 && opModeIsActive()){
            robot.lift.setPower(-0.3);
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

        // Code Review: startTime returns the time the timer was started. It doesn't start the timer.
        preloadTimer.startTime();
        cameraTimer.startTime();
        liftTimer.startTime();
    }

    void _loop() {

        // Code Review: When you move this into the LinearOpMode instead of using this _loop method
        //              which makes it somewhat like Opmode, then you won't need to keep this count
        //              field.
        if(count == 0){
            cameraTimer.reset();
            count++;
        }

        if(autoState == AutoState.CAMERA_WAIT) {
            // Code Review: Do you have to wait 6 seconds? You've also been looking at frames for the
            //              entire duration of Init.
            //              Also note that your xMidVal field is updated all the time, so the decision
            //              you are making is based on the last frame that is seen. You could instead
            //              keep track of the decision from all the frames, or the last 10 frames, or
            //              the last 1 second worth of frames, and make a decision based on voting, etc.
            if(cameraTimer.time(TimeUnit.SECONDS) < 6) {
                // Code Review: You could do this spikeMark decision inside pipeline, instead of
                //              having the logic out here in the opmode.
                if (pipeline.xMidVal < 130 && pipeline.xMidVal > 5) { // Code Review: Why > 5?
                    spikeMark = 1;
                } else if (pipeline.xMidVal < 280 && pipeline.xMidVal > 5) { // Code Review: Why > 5?
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
            // Code Review: Note that preloadTimer is not used. Presumably this would be to give the
            //              servo enough time to fully open? If so, you would want to reset the timer
            //              when you change the servo position anyway.
            preloadTimer.reset();
        } else if(autoState == AutoState.MOVE_1){
            if(!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                moveInches(24);
                // Code Review: Cannot use Thread.sleep! Use a timer!
                // Code Review: Also, why do you want to pause for a second anyway? Can we remove this
                //              for states?
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                strafe(-22,40,40,0,0);
                // Code Review: Need timer / should remove the pause anyway
                try {
                    Thread.sleep(1500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                turnByTicks(600);
                // Code Review: Need timer / should remove the pause anyway
                try {
                    Thread.sleep(1500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                // Code Review: Note that these next two lines will open the servo to drop the pixel
                //              AND start moving at the same time. Is that what you want?
                strafe(3,0,0,0,0);
                robot.preload.setPosition(0.8);
                // Code Review: Need timer / should remove the pause anyway? Or is this a pause for
                //              the servo?
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                // Code Review: Note that these next two lines will open the servo to drop the pixel
                //              AND start moving at the same time. Is that what you want?
                strafe(-3,0,0,0,0);
                robot.preload.setPosition(0.05);
                // Code Review: Need timer / should remove the pause anyway? Or is this a pause for
                //              the servo?
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                moveInches(-31);
                // Code Review: Need timer / should remove the pause anyway?
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                turnByTicks(-110);
                // Code Review: Need timer / should remove the pause anyway?
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                strafe(12,0,0,10,10);
                autoState = AutoState.STOP;
            }
        } else if(autoState == AutoState.MOVE_2){
            // Code Review: Similar comments from MOVE_1 state...
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
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                turnByTicks(-110);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                strafe(6,0,0,5,5);
                autoState = AutoState.STOP;
            }
        } else if(autoState == AutoState.MOVE_3){
            // Code Review: Similar comments from MOVE_1 state...
            if(!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                moveInches(28);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                strafe(-10,0,0,0,0);
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
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                turnByTicks(-60);
                try {
                    Thread.sleep(500);
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