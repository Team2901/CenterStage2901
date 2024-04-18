package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Hardware.CombinedHardware.outtakeLeftClosedPos;
import static org.firstinspires.ftc.teamcode.Hardware.CombinedHardware.outtakeRightClosedPos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Hardware.CombinedHardware;
import org.firstinspires.ftc.teamcode.Hardware.CombinedHardware.*;
import org.firstinspires.ftc.teamcode.OpenCV.ShapeDetection;
import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.util.Blinkin;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Worlds Combined Auto", group = "AAutonomous")
public class WorldsCombinedAuto extends LinearOpMode implements OpenCvCamera.AsyncCameraOpenListener {
    CombinedHardware robot = new CombinedHardware();
    ImprovedGamepad impGamepad1;
    ImprovedGamepad impGamepad2;

    ShapeDetection pipeline = new ShapeDetection(this.telemetry, robot);

    Blinkin lights;
    public static final boolean USE_LIGHTS = true;

    public static final int IMGPROC_SAMPLES = 10;

    public int count = 0;
    public OpenCvCamera camera;
    public int spikeMark;

    public boolean isStopped = false;

    public ElapsedTime cameraTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    public ElapsedTime wristTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    ElapsedTime stall = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public double outtakeLeftOpenPos = CombinedHardware.outtakeLeftOpenPos;
    public double outtakeRightOpenPos = CombinedHardware.outtakeRightOpenPos;

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

        while (opModeInInit()) {
            impGamepad1.update();

            telemetry.addLine("Red (B) or Blue (X)?");
            if (impGamepad1.x.isInitialPress()) {
                robot.alliance = Alliance.BLUE;
            } else if (impGamepad1.b.isInitialPress()) {
                robot.alliance = Alliance.RED;
            }
            telemetry.addData("Alliance", robot.alliance);

            telemetry.addLine("Far (Y) or Close (A)?");
            if (impGamepad1.y.isInitialPress()) {
                robot.startLocation = StartLocation.FAR;
            } else if (impGamepad1.a.isInitialPress()) {
                robot.startLocation = StartLocation.CLOSE;
            }
            telemetry.addData("Starting Location", robot.startLocation);

            telemetry.addLine("Add wait time? (D-pad Up or Down)");
            if (impGamepad1.dpad_up.isInitialPress()) {
                waitSec += 0.5;
            } else if (impGamepad1.dpad_down.isInitialPress() && waitSec >= 0.5) {
                waitSec -= 0.5;
            }
            telemetry.addData("Stall Time", waitSec);

            telemetry.addLine("Park in the Middle (D-pad Left) or Corner (D-pad Right)?");
            if (impGamepad1.dpad_left.isInitialPress()) {
                robot.parkLocation = ParkLocation.MIDDLE;
            } else if (impGamepad1.dpad_right.isInitialPress()) {
                robot.parkLocation = ParkLocation.CORNER;
            }
            telemetry.addData("Parking Location", robot.parkLocation);

            telemetry.addLine();
            telemetry.addData("Spike Mark", pipeline.spikeMark);

            telemetry.update();
        }

        waitForStart();
        moveArm(250);
        stall(0.1);
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
        FtcDashboard.getInstance().startCameraStream(camera, 0);

        robot.imu.resetYaw();
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        autoState = AutoState.CAMERA_WAIT;

        if (USE_LIGHTS) {
            lights = new Blinkin(hardwareMap.get(RevBlinkinLedDriver.class, "blinkinL"), hardwareMap.get(RevBlinkinLedDriver.class, "blinkinR"));
            lights.setPixelStatus(Blinkin.PixelStatus.AUTO);
        }

        robot.outtakeLeft.setPosition(outtakeLeftClosedPos);
        robot.outtakeRight.setPosition(outtakeRightClosedPos);
    }

    long startTime = System.currentTimeMillis();

    //base code from StatesRedClose
    void _loop() {
        if (count == 0) {
            cameraTimer.reset();
            count++;
        }

        if (autoState == AutoState.CAMERA_WAIT) {
//            if (cameraTimer.time(TimeUnit.SECONDS) < 5) {
//                telemetry.addData("Time", cameraTimer.time(TimeUnit.SECONDS));
//                telemetry.addData("X Mid", pipeline.xMidVal);
//                telemetry.addData("Spike Mark", pipeline.spikeMark);
//            } else {
//                autoState = AutoState.CAMERA_DETECTION;
//            }

            //Wait until # of frames processed
            if (camera.getFrameCount() >= IMGPROC_SAMPLES) {
                telemetry.addData("Image processing time", (System.currentTimeMillis() - startTime) / 1000.0);
                telemetry.addData("X Mid", pipeline.xMidVal);
                telemetry.addData("Spike Mark", pipeline.spikeMark);

                spikeMark = pipeline.spikeMark;

                autoState = AutoState.STALL;
                camera.closeCameraDevice();
            }
        } else if (autoState == AutoState.STALL) {
            stall(waitSec);
            autoState = AutoState.CAMERA_DETECTION;
        }else if (autoState == AutoState.CAMERA_DETECTION) {
            if (spikeMark == 1) {
                autoState = AutoState.MOVE_1;
            } else if (spikeMark == 2) {
                autoState = AutoState.MOVE_2;
            } else if (spikeMark == 3) {
                autoState = AutoState.MOVE_3;
            }
        } else if (autoState == AutoState.MOVE_1) {
            if (robot.alliance == Alliance.RED && robot.startLocation == StartLocation.CLOSE) {
                moveInches(28);
                strafe(3);
                //turnByTicks(-1100); //-90 degrees
                robot.turnToAngleAuto(90, this);
                moveInches(2);
                robot.outtakeRight.setPosition(outtakeRightOpenPos);

                stall(2);

                moveInches(-32);

                stall(0.5);

                strafe(9);
                autoState = AutoState.BACKDROP;
            } else if (robot.alliance == Alliance.BLUE && robot.startLocation == StartLocation.CLOSE) {
                moveInches(30);
                strafe(-24);
                //turnByTicks(1070); //90 degrees - a little
                robot.turnToAngleAuto(-90, this);
                robot.outtakeRight.setPosition(outtakeRightOpenPos);

                stall(2);

                moveInches(-11);

                stall(0.5);

                strafe(19); // decreased from 21 after testing once 4/17
                autoState = AutoState.BACKDROP;
            } else if (robot.alliance == Alliance.RED && robot.startLocation == StartLocation.FAR) {
                if (robot.parkLocation == ParkLocation.MIDDLE) {
                    moveInches(40);
                    strafe(-9);
                    //turnByTicks(-2200); //-180 degrees
                    robot.turnToAngleAuto(180, this);

                    robot.outtakeRight.setPosition(outtakeRightOpenPos);
                    stall(2);

                    moveInches(-7.25);
                    //turnByTicks(-850);
                    robot.turnToAngleAuto(-91, this);
                } else {
                    moveInches(21);
                    strafe(-15);
                    robot.outtakeRight.setPosition(outtakeRightOpenPos);
                    stall(2);
                    moveInches(-15);
                    //turnByTicks(1100);
                    robot.turnToAngleAuto(-90, this); //-90 turns clockwise
                    strafe(4.5,0,0,0,0);
                }

                moveInches(91);
                stall(2);

                autoState = AutoState.BACKSTAGE;
            } else { //blue far
                moveInches(29);
                strafe(6);
                //turnByTicks(-1100); //-90 degrees
                robot.turnToAngleAuto(91, this);
                moveInches(6);
                robot.outtakeRight.setPosition(outtakeRightOpenPos);

                stall(2);

                moveInches(-9);

                if (robot.parkLocation == ParkLocation.MIDDLE) {
                    strafe(25);

                    stall(1);
                    robot.turnToAngleAuto(90, this);
                } else {
                    strafe(-33.5);
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
//                turnByTicks(-1100); //-90 degrees
                robot.turnToAngleAuto(90, this);
                moveInches(-32);

                stall(0.5);

                strafe(10);

                autoState = AutoState.BACKDROP;
            } else if (robot.alliance == Alliance.BLUE && robot.startLocation == StartLocation.CLOSE) {
                moveInches(26);
                robot.outtakeRight.setPosition(outtakeRightOpenPos);

                stall(2);

                moveInches(-4);
//                turnByTicks(1070); //90 degrees
                robot.turnToAngleAuto(-90, this);
                moveInches(-31.5);

                stall(0.5);

//                strafe(1.5); // commented out 4/17 after first match

                autoState = AutoState.BACKDROP;
            } else if (robot.alliance == Alliance.RED && robot.startLocation == StartLocation.FAR) {
                if (robot.parkLocation == ParkLocation.MIDDLE) {
                    moveInches(47);
//                    turnByTicks(-2100); //-180
                    robot.turnToAngleAuto(180, this);

                    robot.outtakeRight.setPosition(outtakeRightOpenPos);
                    stall(2);

                    moveInches(-3);
                    stall(1);
//                    turnByTicks(-850);
                    robot.turnToAngleAuto(-92, this);
                    strafe(1,0,0,0,0);
                } else {
                    moveInches(26);

                    robot.outtakeRight.setPosition(outtakeRightOpenPos);
                    stall(2);

                    moveInches(-18);
                    //turnByTicks(1100);
                    robot.turnToAngleAuto(-90, this);
                    strafe(4.0,0,0,0,0);
                }

                moveInches(86);
                stall(2);

                autoState = AutoState.BACKSTAGE;
            } else { //blue far
                if (robot.parkLocation == ParkLocation.MIDDLE) {
                    moveInches(47);
//                    turnByTicks(-2100); //-180
                    robot.turnToAngleAuto(180, this);

                    robot.outtakeRight.setPosition(outtakeRightOpenPos);
                    stall(2);

                    moveInches(-3);
                    stall(1);
//                    turnByTicks(975); //90 degrees
                    robot.turnToAngleAuto(90, this);
                } else {
                    moveInches(27);

                    robot.outtakeRight.setPosition(outtakeRightOpenPos);
                    stall(2);

                    moveInches(-20);
//                    turnByTicks(-850);
                    robot.turnToAngleAuto(90, this);
                    strafe(-3,0,0,0,0);
                }

                moveInches(86);

                stall(1);

                autoState = AutoState.BACKSTAGE;
            }
        } else if (autoState == AutoState.MOVE_3) {
            if (robot.alliance == Alliance.RED && robot.startLocation == StartLocation.CLOSE) {
                moveInches(25);
                strafe(26);
//                turnByTicks(-1100); //-90 degrees
                robot.turnToAngleAuto(90, this);
                robot.outtakeRight.setPosition(outtakeRightOpenPos);

                stall(2);

                moveInches(-10);

                stall(0.5);

                strafe(-3);
                autoState = AutoState.BACKDROP;
            } else if (robot.alliance == Alliance.BLUE && robot.startLocation == StartLocation.CLOSE) {
                moveInches(29);
                strafe(-3);
//                turnByTicks(1100); //90 degrees
                robot.turnToAngleAuto(-90, this);
                moveInches(3);
                robot.outtakeRight.setPosition(outtakeRightOpenPos);

                stall(2);

                moveInches(-32.5);

                stall(0.5);

                strafe(1);
                autoState = AutoState.BACKDROP;
            } else if (robot.alliance == Alliance.RED && robot.startLocation == StartLocation.FAR) {
                moveInches(29);
                strafe(-9);
                //turnByTicks(1100); //90 degrees
                robot.turnToAngleAuto(-90.5, this);
                moveInches(8);
                robot.outtakeRight.setPosition(outtakeRightOpenPos);

                stall(2);

                moveInches(-9);

                if (robot.parkLocation == ParkLocation.MIDDLE) {
                    strafe(-24);

                    stall(1);
                    robot.turnToAngleAuto(-90, this);
                } else {
                    strafe(34.5);
                }

                moveInches(90);

                stall(2);

                autoState = AutoState.BACKSTAGE;
            } else { //blue far
                if (robot.parkLocation == ParkLocation.MIDDLE) {
                    moveInches(40);
                    strafe(16);
//                    turnByTicks(-2200); //-180 degrees
                    robot.turnToAngleAuto(180, this);
                    robot.outtakeRight.setPosition(outtakeRightOpenPos);

                    stall(2);

                    moveInches(-8);
//                    turnByTicks(1150); //90 degrees
                    robot.turnToAngleAuto(90, this);
                } else {
                    moveInches(22);
                    strafe(10);

                    robot.outtakeRight.setPosition(outtakeRightOpenPos);
                    stall(2);

                    moveInches(-16);
//                    turnByTicks(-850);
                    robot.turnToAngleAuto(90, this);
                    strafe(-2.8,0,0,0,0);
                }

                moveInches(91);

                stall(1);

                autoState = AutoState.BACKSTAGE;
            }
        } else if (autoState == AutoState.BACKDROP) {
            robot.rotationServo.setPosition(0.538);

            wristTimer.reset();
            while (wristTimer.time() < 1.5) {
                idle();
            }

            moveArm(3470);

            while (robot.arm.isBusy() && opModeIsActive()) {
                idle();
            }

            robot.outtakeLeft.setPosition(outtakeLeftOpenPos);

            stall(1.5);

            moveArm(250);

            robot.rotationServo.setPosition(0.1);

            if (robot.parkLocation == ParkLocation.CORNER) {
                if (robot.alliance == Alliance.RED) {
                    if (spikeMark == 1) {
                        strafe(-44);
                        moveInches(-6);
                    } else if (spikeMark == 2) {
                        strafe(-37);
                        moveInches(-6);
                    } else if (spikeMark == 3) {
                        strafe(-29);
                        moveInches(-6);
                    }
                } else if (robot.alliance == Alliance.BLUE) {
                    if (spikeMark == 1) {
                        strafe(18);
                    } else if (spikeMark == 2) {
                        strafe(27);
                    } else if (spikeMark == 3) {
                        strafe(38);
                    }
                }
            }

            if (robot.parkLocation == ParkLocation.MIDDLE) {
                if (robot.alliance == Alliance.RED) {
                    if (spikeMark == 1) {
                        strafe(18);
                    } else if (spikeMark == 2) {
                        strafe(28);
                    } else if (spikeMark == 3) {
                        strafe(38);
                    }
                } else if (robot.alliance == Alliance.BLUE) { // has not been tested yet
                    if (spikeMark == 1) {
                        strafe(-38);
                    } else if (spikeMark == 2) {
                        strafe(-30);
                    } else if (spikeMark == 3) {
                        strafe(-20);
                    }
                }
            }

            autoState = AutoState.STOP;

        } else if (autoState == AutoState.BACKSTAGE) {
            if (robot.alliance == Alliance.RED && robot.parkLocation == ParkLocation.MIDDLE) {
                turnByTicks(95);
            } else if (robot.alliance == Alliance.BLUE && robot.parkLocation == ParkLocation.MIDDLE) {
                turnByTicks(-95);
            }

            robot.outtakeLeft.setPosition(outtakeLeftOpenPos);

            stall(2);

            robot.arm.setTargetPosition(300);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm.setPower(0.6);

            autoState = AutoState.STOP;
        } else if (autoState == AutoState.STOP) {
            robot.setDrivePower(0);
            isStopped = true;
        }

        telemetry.addData("Spike Mark", pipeline.spikeMark);
        telemetry.addData("STATE", autoState);
        telemetry.update();

        lights.update();
    }

    private void moveInches(double inches) {
        int ticks = (int) (inches * robot.TICKS_PER_INCH);

        robot.frontLeft.setTargetPosition(robot.frontLeft.getCurrentPosition() + ticks);
        robot.frontRight.setTargetPosition(robot.frontRight.getCurrentPosition() + ticks);
        robot.backLeft.setTargetPosition(robot.backLeft.getCurrentPosition() + ticks);
        robot.backRight.setTargetPosition(robot.backRight.getCurrentPosition() + ticks);

        robot.setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setDrivePower(0.35);

        while (opModeIsActive() && robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy()) {
//            telemetry.addData("Current Left Position", robot.backLeft.getCurrentPosition());
//            telemetry.addData("Current Right
//            public void onError ( int errorCode){
//
//            }
        }
    }

    //overloaded method so 0,0,0,0 isn't everywhere
    private void strafe(double inches) {
        strafe(inches, 0, 0, 0, 0);
    }

    private void strafe(double inches, int FLTicks, int FRTicks, int BLTicks, int BRTicks) {
        int ticks = (int) (inches * robot.TICKS_PER_INCH);

        robot.frontLeft.setTargetPosition(robot.frontLeft.getCurrentPosition() + ticks + FLTicks);
        robot.frontRight.setTargetPosition(robot.frontRight.getCurrentPosition() - ticks - FRTicks);
        robot.backLeft.setTargetPosition(robot.backLeft.getCurrentPosition() - ticks - BLTicks);
        robot.backRight.setTargetPosition(robot.backRight.getCurrentPosition() + ticks + BRTicks);

        robot.setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setDrivePower(0.35);

        while (opModeIsActive() && robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy()) {
//            telemetry.addData("Current Left Position", robot.frontLeft.getCurrentPosition());
//            telemetry.addData("Current Right Position", robot.frontRight.getCurrentPosition());
        }

        robot.setDrivePower(0);

        robot.setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        robot.setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setDrivePower(0.35);

        while (opModeIsActive() && robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy()) {
//            telemetry.addData("Current Left Position", robot.frontLeft.getCurrentPosition());
//            telemetry.addData("Current Right Position", robot.frontRight.getCurrentPosition());
        }
    }

    /**
     * Turn by degrees, should turn same direction as turnByTicks
     * Might overshoot and flip directions a little
     *
     * @param theta degrees to turn by
     */

    public void turnByDegrees(double theta) {
        double pwr = 0.35;

        double currentAngle = getIMUYaw();

        double targetAngle = currentAngle + theta;

        double distance = 100;
        double dtheta;

        robot.setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (Math.abs(distance) > 1 && opModeIsActive()) {
            //using delta so angles are less annoying. dtheta > 10 means jump from +- 180 to -+ 180
            dtheta = getIMUYaw() - currentAngle;
            if (dtheta < 10) currentAngle += dtheta;

            distance = targetAngle - currentAngle;

            double sign = Math.signum(distance);

            robot.frontLeft.setPower(pwr * sign);
            robot.frontRight.setPower(-pwr * sign);
            robot.backLeft.setPower(pwr * sign);
            robot.backRight.setPower(-pwr * sign);
        }

        robot.setDrivePower(0);

    }

    /**
     * @return The side-to-side lateral rotation of the robot (rotation around the Z axis), normalized to the range of [-180,+180) degrees.
     */

    public double getIMUYaw() {
        YawPitchRollAngles angles = robot.imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.DEGREES);
    }

    public void moveArm(int ticks) {
        robot.arm.setTargetPosition(ticks);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(0.65);
    }

    public void stall(double seconds) {
        stall.reset();
        while (opModeIsActive() && stall.time() < seconds) {
            idle();
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