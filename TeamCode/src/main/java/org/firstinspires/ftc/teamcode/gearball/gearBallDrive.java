package org.firstinspires.ftc.teamcode.gearball;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.bubblecloud.vecmath.Vector3f;
import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;

@TeleOp(name = "gearBallDrive", group = "AAAtest")
public class gearBallDrive extends LinearOpMode {
    public boolean isStopped = false;

    ImprovedGamepad impGamepad1;
    ImprovedGamepad impGamepad2;

    GearBallHardware hardware = new GearBallHardware();
    @Override
    public void runOpMode() throws InterruptedException {
        impGamepad1 = new ImprovedGamepad(gamepad1, new ElapsedTime(), "gamepad1");
        impGamepad2 = new ImprovedGamepad(gamepad2, new ElapsedTime(), "gamepad2");
        hardware.init(hardwareMap);
        hardware.setTelemetry(telemetry);
        hardware.setGamepad(impGamepad1);
        waitForStart();
        while (opModeIsActive()) {
            if (isStopped) {
                break;
            } else {
                _loop();
            }
        }
    }
    double factor = 1;

//    Vector3f targetVector = new Vector3f(0,1,0);

    void _loop(){
        impGamepad1.update();
        impGamepad2.update();

        if (impGamepad1.y.getValue()) {
            factor = 1;
        }
        if (impGamepad1.b.getValue()) {
            factor = 0.25;
        }
        if (impGamepad1.a.getValue()) {
            factor = 0.5;
        }
        if (impGamepad1.x.getValue()) {
            factor = 0.75;
        }

        if(impGamepad1.right_stick_button.getValue()){
//        if (impGamepad1.right_stick_x.getValue() != 0 || impGamepad1.right_stick_y.getValue() != 0) {
//            targetVector.x += impGamepad1.right_stick_x.getValue();
//            targetVector.y += impGamepad1.right_stick_x.getValue();
            hardware.update(impGamepad1.right_stick_x.getValue(),impGamepad1.right_stick_y.getValue());
            sleep(1);
        }
    }
}
