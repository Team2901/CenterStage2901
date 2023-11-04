package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.MecanumDriveHardware;
import org.firstinspires.ftc.teamcode.Vision.ShapeDetectionBlue;

@Autonomous(name = "Shape Detection Blue", group = "Autonomous")
public class MecanumAutoShapesBlue extends LinearOpMode {

    MecanumDriveHardware robot = new MecanumDriveHardware();
    ShapeDetectionBlue bluePipeline = new ShapeDetectionBlue(this.telemetry);

//    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public int spikeMark = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap, telemetry);

        while(!opModeIsActive()) {
            if(bluePipeline.xCoord() > 0) {
                if (bluePipeline.xCoord() < 107) {
                    spikeMark = 1;
                } else if (bluePipeline.xCoord() < 214) {
                    spikeMark = 2;
                } else if (bluePipeline.xCoord() < 320) {
                    spikeMark = 3;
                }
            }

            telemetry.addData("X", bluePipeline.xCoord());
            telemetry.addData("Spike Mark", spikeMark);
            telemetry.update();
        }

        waitForStart();

        //move forward, strafe (if 1 or 2), deposit purple, move back, strafe left to park, deposit yellow (for BLUE side)
//        timer.reset();
        if(spikeMark == 1){
            moveInches(12);
            strafe(-12);
            robot.preload.setPosition(0);
            moveInches(-12);
            strafe(-24);
        } else if(spikeMark == 2){
            moveInches(24);
            robot.preload.setPosition(0);
            moveInches(-24);
            strafe(-36);
            moveInches(24);
        } else if(spikeMark == 3){
            moveInches(12);
            strafe(12);
            robot.preload.setPosition(0);
            moveInches(-12);
            strafe(-48);
            moveInches(48);
        }
//        robot.intake.setPower(-0.8);
    }

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

        while(opModeIsActive() && (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy())){
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

        while(opModeIsActive() && (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy())){
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
}