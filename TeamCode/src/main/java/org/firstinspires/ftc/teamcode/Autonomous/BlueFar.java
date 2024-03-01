package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.OpenCV.ShapeDetection;
import org.firstinspires.ftc.teamcode.TeleOp.StatesTeleOp;
import org.openftc.easyopencv.OpenCvCamera;
import java.util.concurrent.TimeUnit;

@Autonomous (name = "States Blue Far", group = "AAutonomous")
public class BlueFar extends AutoOpMode implements OpenCvCamera.AsyncCameraOpenListener {

    ShapeDetection pipeline = new ShapeDetection("blue",this.telemetry);

    public double xMidInit = 888;
    public boolean isStopped = false;

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
        } else if (autoState == AutoState.MOVE_1) {
            if (!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                moveInches(31);
                strafe(6, 0, 0, 0, 0);
                turnByTicks(-1100); //-90 degrees
                moveInches(6);
                robot.outtakeRight.setPosition(outtakeRightOpenPos);

                stall.reset();
                while (stall.time() < 2) {
                    idle();
                }

                moveInches(-9);
                strafe(21.5, 0, 0, 0, 0);

                stall.reset();
                while (stall.time() < 1) {
                    idle();
                }

                turnByTicks(75);
                moveInches(90);

                stall.reset();
                while (stall.time() < 1) {
                    idle();
                }

                autoState = AutoState.BACKSTAGE;
            }
        } else if (autoState == AutoState.MOVE_2) {
            if (!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                moveInches(43);
                turnByTicks(-2100); //-180
                robot.outtakeRight.setPosition(outtakeRightOpenPos);

                stall.reset();
                while (stall.time() < 2) {
                    idle();
                }

                moveInches(-3);

                stall.reset();
                while (stall.time() < 1) {
                    idle();
                }

                turnByTicks(975); //90 degrees
                moveInches(86);

                stall.reset();
                while (stall.time() < 1) {
                    idle();
                }

                autoState = AutoState.BACKSTAGE;
            }
        } else if (autoState == AutoState.MOVE_3) {
            if (!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
                moveInches(40);
                strafe(14, 0, 0, 0, 0);
                turnByTicks(-2200); //-180 degrees
                robot.outtakeRight.setPosition(outtakeRightOpenPos);

                stall.reset();
                while (stall.time() < 2) {
                    idle();
                }

                moveInches(-8);
                turnByTicks(1150); //90 degrees
                moveInches(91);

                stall.reset();
                while (stall.time() < 1) {
                    idle();
                }

                autoState = AutoState.BACKSTAGE;
            }
        } else if (autoState == AutoState.BACKSTAGE) {
            if (!robot.frontLeft.isBusy() && !robot.frontRight.isBusy() && !robot.backLeft.isBusy() && !robot.backRight.isBusy()) {
//                turnByTicks(2400); //more than 180
                turnByTicks(-95);
                robot.outtakeLeft.setPosition(outtakeLeftOpenPos);

                stall.reset();
                while (stall.time() < 2) {
                    idle();
                }

                robot.arm.setTargetPosition(300);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(0.6);

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
