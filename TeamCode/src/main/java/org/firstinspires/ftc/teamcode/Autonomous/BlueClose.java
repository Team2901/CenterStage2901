package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.OpenCV.ShapeDetection;
import org.firstinspires.ftc.teamcode.TeleOp.StatesTeleOp;
import org.openftc.easyopencv.OpenCvCamera;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "States Blue Close", group = "AAutonomous")
public class BlueClose extends AutoOpMode implements OpenCvCamera.AsyncCameraOpenListener {

    ShapeDetection pipeline = new ShapeDetection("blue", this.telemetry);
    public double xMidInit = 888;
    public int spikeMark;
    public boolean outtakeRightClosed = false;
    public boolean outtakeLeftClosed = false;

    public double outtakeLeftClosedPos = StatesTeleOp.outtakeLeftClosedPos;
    public double outtakeLeftOpenPos = StatesTeleOp.outtakeLeftOpenPos;
    public double outtakeRightClosedPos = StatesTeleOp.outtakeRightClosedPos;
    public double outtakeRightOpenPos = StatesTeleOp.outtakeRightOpenPos;

    public int currentArmTicks = 0;
    double initArmAngle = 60.0;
    double armAngle = initArmAngle;

    void _loop() {
        if (autoState == AutoState.CAMERA_WAIT) {
            if (cameraTimer.time(TimeUnit.SECONDS) < 5) {
                telemetry.addData("Time", cameraTimer.time(TimeUnit.SECONDS));
                telemetry.addData("X Mid", pipeline.xMidVal);
                telemetry.addData("Spike Mark", pipeline.spikeMark);
            } else {
                autoState = AutoState.CAMERA_DETECTION;
            }
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
            if (!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                moveInches(30);
                strafe(-24, 0, 0, 0, 0);
                turnByTicks(1070); //90 degrees - a little
                robot.outtakeRight.setPosition(outtakeRightOpenPos);

                stall.reset();
                while (stall.time() < 2) {
                    idle();
                }

                moveInches(-11);

                stall.reset();
                while (stall.time() < 2) {
                    idle();
                }

                strafe(21, 0, 0, 0, 0);
                autoState = AutoState.BACKDROP;
            }
        } else if (autoState == AutoState.MOVE_2) {
            if (!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                moveInches(27);
                robot.outtakeRight.setPosition(outtakeRightOpenPos);

                stall.reset();
                while (stall.time() < 2) {
                    idle();
                }

                moveInches(-4);
                turnByTicks(1070); //90 degrees
                moveInches(-32.5);

                stall.reset();
                while (stall.time() < 2) {
                    idle();
                }

                strafe(3, 0, 0, 0, 0);
                turnByTicks(30);

                autoState = AutoState.BACKDROP;
            }
        } else if (autoState == AutoState.MOVE_3) {
            if (!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                moveInches(31);
                strafe(-3, 0, 0, 0, 0);
                turnByTicks(1100); //90 degrees
                moveInches(3);
                robot.outtakeRight.setPosition(outtakeRightOpenPos);

                stall.reset();
                while (stall.time() < 2) {
                    idle();
                }

                moveInches(-32.5);

                stall.reset();
                while (stall.time() < 1) {
                    idle();
                }

                strafe(3, 0, 0, 0, 0);
                turnByTicks(-80);
                autoState = AutoState.BACKDROP;
            }
        } else if (autoState == AutoState.BACKDROP) {
            if (!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                robot.rotationServo.setPosition(0.495);

                wristTimer.reset();
                while (wristTimer.time() < 1.5) {
                    idle();
                }

                robot.arm.setTargetPosition(2280);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(0.65);

                while (robot.arm.isBusy() && opModeIsActive()) {
                    idle();
                }

                robot.outtakeLeft.setPosition(outtakeLeftOpenPos);

                stall.reset();
                while (stall.time() < 3) {
                    idle();
                }

                robot.arm.setTargetPosition(150);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(0.65);

                robot.rotationServo.setPosition(0.1);

                //to move into the corner
                if (spikeMark == 1) {
                    strafe(18, 0, 0, 0, 0);
                } else if (spikeMark == 2) {
                    strafe(28, 0, 0, 0, 0);
                } else if (spikeMark == 3) {
                    strafe(38, 0, 0, 0, 0);
                }

                autoState = AutoState.STOP;
            }
        } else if (autoState == AutoState.STOP) {
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
}
