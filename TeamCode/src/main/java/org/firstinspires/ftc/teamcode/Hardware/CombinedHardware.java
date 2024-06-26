package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Utilities.ConfigUtilities;

@Config
public class CombinedHardware {

    public static final double TICKS_PER_MOTOR_REV = 537.7;
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * 3.78;
    public static final double DRIVE_GEAR_RATIO = 1;
    public static final double TICKS_PER_DRIVE_REV = TICKS_PER_MOTOR_REV * DRIVE_GEAR_RATIO;
    public static final double TICKS_PER_INCH = TICKS_PER_DRIVE_REV / WHEEL_CIRCUMFERENCE;

    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor arm;
    public Servo outtakeLeft;
    public Servo outtakeRight;
    public Servo planeServo;
    public Servo rotationServo;
    public DistanceSensor distanceSensorLeft;
    public DistanceSensor distanceSensorRight;
    public ColorRangeSensor colorSensorLeft;
    public ColorRangeSensor colorSensorRight;

    public IMU imu;

    // all init values below are set for the states robot
    public static double outtakeLeftClosedPos = 0.225;
    public static double outtakeLeftOpenPos = 0.03;
    public static double outtakeRightClosedPos = 0.230;
    public static double outtakeRightOpenPos = 0.03;

//    public static final double maxOuttakeLeftOpenPos = 0.315 - 0.175;
//    public static final double maxOuttakeRightOpenPos = 0.3 - 0.175;

    public static int maxHeightArmTicks = 3800; //preset deliver point
    public static int maxArmTicks = 5500;
    public static int minArmTicks = 15;
    public int initArmAngle = 60;

    public static int boundingLine1 = 40;
    public static int boundingLine2 = 200;

    public static double rotationServoInitPos;
    public static double rotationServoMin;

    // idk what this is on the states robot (i think it was 1?)
    public static double planeServoReleasePos;

    public static double colorSensorPixelDistance = 0.35;

    public enum Alliance {
        RED,
        BLUE
    }

    public Alliance alliance = Alliance.RED;

    public enum StartLocation {
        FAR,
        CLOSE
    }

    public StartLocation startLocation = StartLocation.FAR;

    public enum Config {
        WORLDS,
        STATES
    }

    public Config config = Config.WORLDS;

    public enum ParkLocation {
        MIDDLE,
        CORNER
    }

    public ParkLocation parkLocation = ParkLocation.CORNER;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        // initialize motors
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        arm = hardwareMap.dcMotor.get("arm");
        outtakeLeft = hardwareMap.servo.get("outtakeLeft");
        outtakeRight = hardwareMap.servo.get("outtakeRight");
        planeServo = hardwareMap.servo.get("planeServo");
        rotationServo = hardwareMap.servo.get("rotationServo");
        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "distanceSensorLeft");
        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "distanceSensorRight");
        colorSensorLeft = hardwareMap.get(ColorRangeSensor.class, "colorSensorLeft");
        colorSensorRight = hardwareMap.get(ColorRangeSensor.class, "colorSensorRight");

        // set motor directions (so it doesn't perpetually rotate)
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        // stop motors during initialization
        setDrivePower(0);
        arm.setPower(0);

        // reset encoder positions to 0
        setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set motors up to run with encoders
        //backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // better braking
        setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set up imu
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbFacingDirection);
        IMU.Parameters IMUParameters = new IMU.Parameters(orientationOnRobot);
        imu.initialize(IMUParameters);

        String configName = ConfigUtilities.getRobotConfigurationName().toLowerCase();
        if (configName.contains("worlds")) {
            arm.setDirection(DcMotorSimple.Direction.FORWARD);
            maxArmTicks = 3900;
            maxHeightArmTicks = 3100;
            minArmTicks = 220;
            rotationServoInitPos = .125;
            rotationServoMin = 0.125;
            //set constants for worlds at the top
//            outtakeLeftClosedPos = 0.315;
//            outtakeLeftOpenPos = outtakeLeftClosedPos - 0.175;
//            outtakeRightClosedPos = 0.425;
//            outtakeRightOpenPos = outtakeRightClosedPos - 0.175;
            planeServoReleasePos = 0.1; //when loaded, pos is > planeServoReleasePos
            boundingLine1 = 130;
            boundingLine2 = 300;
            initArmAngle = 70;
        } else {
            arm.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public double getAngle() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return AngleUnit.normalizeDegrees(angles.getYaw(AngleUnit.DEGREES));
    }

    public double recalculateAngle() {
        double calculatedAngle;
        calculatedAngle = ((180 * arm.getCurrentPosition()) / 3450) + initArmAngle;
        return calculatedAngle;
    }

    public double turnToAngle(double turnAngle) {

        //robot.getAngle is between -180 and 180, starting at 0
        double turnPower = 0;
        double targetAngle = AngleUnit.normalizeDegrees(turnAngle);
        double startAngle = getAngle();
        double turnError = AngleUnit.normalizeDegrees(targetAngle - startAngle);
        if (Math.abs(turnError) > 0.5) turnPower = Math.max(Math.min(turnError / 50, 0.75), -0.75);

        return turnPower;
    }

    public double snapToAngle(double turnAngle) {
        double targetAngle = Math.round(getAngle()/turnAngle) * turnAngle;
        return turnToAngle(targetAngle);
    }


        public void turnToAngleAuto(double turnAngle, LinearOpMode opMode) {
        double turnPower = 0;
        double targetAngle = AngleUnit.normalizeDegrees(turnAngle);
        double startAngle = getAngle();
        double turnError = AngleUnit.normalizeDegrees(targetAngle - startAngle);

        DcMotor.RunMode originalMode = frontLeft.getMode();
        // TODO: each motor?...

        setDrivePower(0.0);
        setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opMode.opModeIsActive() && /*!(turnError < .5 && turnError > -.5)*/ Math.abs(turnError) > 0.5) {
//            if (turnError >= 0) {
//                turnPower = turnError / 50;
//                if (turnPower > .75) {
//                    turnPower = .75;
//                }
//            } else if (turnError < 0) {
//                turnPower = turnError / 50;
//                if (turnPower < -.75) {
//                    turnPower = -.75;
//                }
//            }
            turnPower = Math.max(Math.min(turnError / 50, 0.75), -0.75);

            frontLeft.setPower(-turnPower);
            frontRight.setPower(turnPower);
            backLeft.setPower(-turnPower);
            backRight.setPower(turnPower);

            double currentAngle = getAngle();
            turnError = AngleUnit.normalizeDegrees(targetAngle - currentAngle);
        }

        setDrivePower(0.0);
        setDriveRunMode(originalMode);
    }

    public void adjustWrist() {
        double armAngle = recalculateAngle();
        if (armAngle < 75) {
            rotationServo.setPosition(0.075); //increased by 0.01 1:44
            //when arm angle increases, servo angle decreases
        } else if (armAngle < 93) { //90 degrees, but a little bit of error in the math so have to adjust manually
            rotationServo.setPosition(0.1);
            //servo angle = arm angle + 25
            //ethan says "zone between 65 and 90 that keeps the servo at the arm angle+25 degrees (so it is as close to horizontal as possible)"
        } else if (armAngle < 200) {
            rotationServo.setPosition(0.325);
        } else if (armAngle < 270) {
            rotationServo.setPosition(0.75 - ((armAngle - 200) * 0.004));
        }
    }

    public static double approachingSpeed = 0.35;
    public static double slowingDistance = 6;
    public static double backdropOffset = 1.5;

    public double approachBackdrop() {
        double drivePower = 0;
        double targetDistance = backdropOffset;
        double currentDistance = distanceSensorLeft.getDistance(DistanceUnit.INCH);
        double distanceError = currentDistance - targetDistance;
        double kp = (approachingSpeed / slowingDistance);

        drivePower = distanceError * kp;
        if (drivePower > approachingSpeed) {
            drivePower = approachingSpeed;
        }

        return drivePower;
    }

    public void setDriveRunMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    public void setDrivePower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    public void setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        frontLeft.setZeroPowerBehavior(zeroPowerBehavior);
        frontRight.setZeroPowerBehavior(zeroPowerBehavior);
        backLeft.setZeroPowerBehavior(zeroPowerBehavior);
        backRight.setZeroPowerBehavior(zeroPowerBehavior);
    }
}
