package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.util.Blinkin;

@TeleOp(name = "blinkintest", group = "AAAAA_test")
public class BlinkinTest extends OpMode {
    ImprovedGamepad impGamepad1;
    ImprovedGamepad impGamepad2;
    Blinkin blinkin;

    public void init() {
        impGamepad1 = new ImprovedGamepad(gamepad1, new ElapsedTime(), "gamepad1");
        impGamepad2 = new ImprovedGamepad(gamepad2, new ElapsedTime(), "gamepad2");

        blinkin = new Blinkin(hardwareMap.get(RevBlinkinLedDriver.class, "blinkinL"), hardwareMap.get(RevBlinkinLedDriver.class, "blinkinR"));
    }

//    int patternNumber = 0;

    public void loop() {
        impGamepad1.update();
        impGamepad2.update();
//        if(impGamepad1.a.isInitialPress()){
//            patternNumber++;
//            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(patternNumber));
//        }else if(impGamepad1.y.isInitialPress()){
//            patternNumber--;
//            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(patternNumber));
//        }
//        telemetry.addData("pattern", patternNumber);
//        telemetry.update();

        if (impGamepad1.left_bumper.getValue()) {
            blinkin.setPixelLeftStatus(Blinkin.PixelStatus.REQUESTED);
            if (impGamepad1.a.isInitialPress()) {
                blinkin.setPixelLeft(Blinkin.PixelColor.GREEN);
            }
            if (impGamepad1.y.isInitialPress()) {
                blinkin.setPixelLeft(Blinkin.PixelColor.YELLOW);
            }
            if (impGamepad1.x.isInitialPress()) {
                blinkin.setPixelLeft(Blinkin.PixelColor.PURPLE);
            }
            if (impGamepad1.b.isInitialPress()) {
                blinkin.setPixelLeft(Blinkin.PixelColor.WHITE);
            }
        }
        if (impGamepad1.right_bumper.getValue()) {
            blinkin.setPixelRightStatus(Blinkin.PixelStatus.REQUESTED);
            if (impGamepad1.a.isInitialPress()) {
                blinkin.setPixelRight(Blinkin.PixelColor.GREEN);
            }
            if (impGamepad1.y.isInitialPress()) {
                blinkin.setPixelRight(Blinkin.PixelColor.YELLOW);
            }
            if (impGamepad1.b.isInitialPress()) {
                blinkin.setPixelRight(Blinkin.PixelColor.WHITE);
            }
            if (impGamepad1.x.isInitialPress()) {
                blinkin.setPixelRight(Blinkin.PixelColor.PURPLE);
            }
        }
        if (impGamepad1.left_trigger.getValue() > 0.5) {
            blinkin.setPixelLeftStatus(Blinkin.PixelStatus.EMPTY);
        } else if (impGamepad1.right_trigger.getValue() > 0.5) {
            blinkin.setPixelRightStatus(Blinkin.PixelStatus.EMPTY);
        }
        if(impGamepad1.left_stick.click.isInitialPress()){
            blinkin.setPixelLeftStatus(Blinkin.PixelStatus.HELD);
        }else if(impGamepad1.right_stick.click.isInitialPress()){
            blinkin.setPixelRightStatus(Blinkin.PixelStatus.HELD);
        }
        blinkin.update();
    }
}
