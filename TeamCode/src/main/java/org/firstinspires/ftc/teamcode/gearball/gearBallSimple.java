package org.firstinspires.ftc.teamcode.gearball;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
@TeleOp(name = "gearBallSimple", group = "AAAtest")
public class gearBallSimple extends LinearOpMode {
    public boolean isStopped = false;

    ImprovedGamepad impGamepad1;
    ImprovedGamepad impGamepad2;

    GBMPGear gearA;
    GBMPGear gearB;

    Telemetry telemetryD;

    @Override
    public void runOpMode() throws InterruptedException {
//        new debugGraph();
        impGamepad1 = new ImprovedGamepad(gamepad1, new ElapsedTime(), "gamepad1");
        impGamepad2 = new ImprovedGamepad(gamepad2, new ElapsedTime(), "gamepad2");
        gearA = new GBMPGear(this.hardwareMap.crservo.get("axservo0"), this.hardwareMap.crservo.get("axservo1"), this.hardwareMap.analogInput.get("axservopos0"), this.hardwareMap.analogInput.get("axservopos1"));
        gearB = new GBMPGear(this.hardwareMap.crservo.get("axservo2"), this.hardwareMap.crservo.get("axservo3"), this.hardwareMap.analogInput.get("axservopos2"), this.hardwareMap.analogInput.get("axservopos3"));
        gearA.setTelemetry(telemetry);
        gearA.setGamepad(impGamepad1);
        gearB.setTelemetry(telemetry);
        gearB.setGamepad(impGamepad1);
        waitForStart();

//        roll(Math.PI/2);
//        moveToTarget();

        while (opModeIsActive()) {
            if (isStopped) {
                break;
            } else {
                _loop();
            }
        }
    }

    public void _loop(){
        impGamepad1.update();
        impGamepad2.update();
        if(impGamepad1.a.isInitialPress()){
            pitch(Math.PI/2);
        }
        if(impGamepad1.y.isInitialPress()){
            pitch(-Math.PI/2);
        }
        if(impGamepad1.x.isInitialPress()){
            roll(Math.PI/2);
        }
        if(impGamepad1.b.isInitialPress()){
            roll(-Math.PI/2);
        }
    }

    public void roll(double theta){
        gearA.setTargetOrientation(gearA.pitch,gearA.roll + theta);
        gearB.setTargetOrientation(gearB.pitch,gearB.roll + theta);

        moveToTarget();
    }

    public void pitch(double theta){
        gearA.setTargetOrientation(gearA.pitch + theta,gearA.roll);
        gearB.setTargetOrientation(gearB.pitch + theta,gearB.roll);

        moveToTarget();
    }
    public void moveToTarget(){
        boolean aDone = false;
        boolean bDone = false;
//        bDone = true;

        while (!aDone || !bDone) {
            if (impGamepad1 != null && impGamepad1.left_stick_button.isInitialPress()) break;
            aDone = gearA.moveToTargetOrientation();
            bDone = gearB.moveToTargetOrientation();
//            telemetry.addData("ADone",aDone);
//            telemetry.addData("bDone",bDone);
//            telemetry.update();
        }
    }
}
