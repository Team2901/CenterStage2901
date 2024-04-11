package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.CombinedHardware;
import org.firstinspires.ftc.teamcode.OpenCV.ShapeDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous (name = "Testing Camera Functionality with 4 Sensors", group = "AAutonomous")
public class CameraTestingWithSensors extends LinearOpMode implements OpenCvCamera.AsyncCameraOpenListener {
    CombinedHardware robot = new CombinedHardware();

    ShapeDetection pipeline = new ShapeDetection(this.telemetry);

    public OpenCvCamera camera;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(this.hardwareMap, telemetry);
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewID);
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(this);

        FtcDashboard.getInstance().startCameraStream(camera, 0);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Spike Mark", pipeline.spikeMark);
            telemetry.addData("Left Distance (in)", robot.distanceSensorLeft.getDistance(DistanceUnit.INCH));
            telemetry.addData("Right Distance (in)", robot.distanceSensorRight.getDistance(DistanceUnit.INCH));
            telemetry.addData("Left Color RED", robot.colorSensorLeft.red());
            telemetry.addData("Left Color GREEN", robot.colorSensorLeft.green());
            telemetry.addData("Left Color BLUE", robot.colorSensorLeft.blue());
            telemetry.addData("Right Color RED", robot.colorSensorRight.red());
            telemetry.addData("Right Color GREEN", robot.colorSensorRight.green());
            telemetry.addData("Right Color BLUE", robot.colorSensorRight.blue());
            telemetry.update();
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
