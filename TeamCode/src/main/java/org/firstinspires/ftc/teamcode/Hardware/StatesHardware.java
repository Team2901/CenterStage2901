package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class StatesHardware{

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

    public IMU imu;

    public static double outtakeLeftClosedPos = 0.315;
    public static double outtakeLeftOpenPos = outtakeLeftClosedPos - 0.2;
    public static double outtakeRightClosedPos = 0.3;
    public static double outtakeRightOpenPos = outtakeRightClosedPos - 0.2;

    public static int maxHeightArmTicks = 3390; //preset deliver point
    public static int maxArmTicks = 5500;
    public static int minArmTicks = 15;
    public int initArmAngle = 60;

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

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
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

        // set motor directions (so it doesn't perpetually rotate)
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        // stop motors during initialization
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        arm.setPower(0);

        // reset encoder positions to 0
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set motors up to run with encoders
        //backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // better braking
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        imu.resetYaw();
    }

    public double getAngle(){
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return AngleUnit.normalizeDegrees(angles.getYaw(AngleUnit.DEGREES));
    }

    public double recalculateAngle(){
        double calculatedAngle;
        calculatedAngle = ((210*arm.getCurrentPosition())/5360) + initArmAngle;
        return calculatedAngle;
    }

    public double turnToAngle(double turnAngle){

        //robot.getAngle is between -180 and 180, starting at 0
        double turnPower = 0;
        double targetAngle = AngleUnit.normalizeDegrees(turnAngle);
        double startAngle = getAngle();
        double turnError = AngleUnit.normalizeDegrees(targetAngle - startAngle);
        if(!(turnError < .5 && turnError > -.5)){
            if(turnError >= 0){
                turnPower = turnError/50;
                if(turnPower > .75){
                    turnPower = .75;
                }
            }else if(turnError < 0){
                turnPower = turnError/50;
                if(turnPower < -.75){
                    turnPower = -.75;
                }
            }
//            robot.frontLeft.setPower(-turnPower);
//            robot.frontRight.setPower(turnPower);
//            robot.backLeft.setPower(-turnPower);
//            robot.backRight.setPower(turnPower);

            double currentAngle = getAngle();
            turnError = AngleUnit.normalizeDegrees(targetAngle - currentAngle);
        }
//        robot.frontLeft.setPower(0);
//        robot.frontRight.setPower(0);
//        robot.backRight.setPower(0);
//        robot.backLeft.setPower(0);

        return turnPower;
    }
    public void adjustWrist(){
        double armAngle = recalculateAngle();
        if(armAngle < 75){
            rotationServo.setPosition(0.125);
            //when arm angle increases, servo angle decreases
        } else if (armAngle < 93){ //90 degrees, but a little bit of error in the math so have to adjust manually
            rotationServo.setPosition(0.1);
            //servo angle = arm angle + 25
            //ethan says "zone between 65 and 90 that keeps the servo at the arm angle+25 degrees (so it is as close to horizontal as possible)"
        } else if(armAngle < 190){
            rotationServo.setPosition(0.325);
        } else if(armAngle < 270){
            rotationServo.setPosition(0.75 - ((armAngle - 190) * 0.004));
        }
    }
}
