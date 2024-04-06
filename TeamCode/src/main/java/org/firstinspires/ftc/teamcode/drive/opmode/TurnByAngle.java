package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Hardware.StatesHardware;
import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;

@TeleOp(name = "TurnByDegreesTest", group = "AAATEST")
public class TurnByAngle extends LinearOpMode {
    ImprovedGamepad impGamepad1;
    ImprovedGamepad impGamepad2;
    StatesHardware robot = new StatesHardware();

    public boolean isStopped = false;

    public void runOpMode() throws InterruptedException {
        impGamepad1 = new ImprovedGamepad(gamepad1, new ElapsedTime(), "gamepad1");
        impGamepad2 = new ImprovedGamepad(gamepad1, new ElapsedTime(), "gamepad2");
        robot.init(this.hardwareMap, telemetry);

        while (opModeIsActive()) {
            if (isStopped) {
                break;
            } else {
                _loop();
            }
        }
    }

    public int angle = 0;

    public void _loop() {
        impGamepad1.update();
        impGamepad2.update();

        if (impGamepad1.a.isInitialPress()) {
            angle += 5;
        }
        if (impGamepad1.y.isInitialPress()) {
            angle -= 5;
        }

        if (impGamepad1.left_stick.click.isInitialPress()) {
            turnByDegrees(angle);
        }

        telemetry.addData("Turn Angle", angle);
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
}
