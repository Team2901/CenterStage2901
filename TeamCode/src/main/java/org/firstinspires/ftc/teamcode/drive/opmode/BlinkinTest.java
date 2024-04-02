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

        blinkin = new Blinkin(hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"));
    }

    public void loop() {
        impGamepad1.update();
        impGamepad2.update();

        if(impGamepad1.left_bumper.getValue()){
            if(impGamepad1.a.isInitialPress()){
                blinkin.setPixel1(Blinkin.PixelColor.GREEN);
            }
            if(impGamepad1.y.isInitialPress()){
                blinkin.setPixel1(Blinkin.PixelColor.YELLOW);
            }
            if(impGamepad1.x.isInitialPress()){
                blinkin.setPixel1(Blinkin.PixelColor.WHITE);
            }
        }
        if(impGamepad1.right_bumper.getValue()){
            if(impGamepad1.a.isInitialPress()){
                blinkin.setPixel2(Blinkin.PixelColor.GREEN);
            }
            if(impGamepad1.y.isInitialPress()){
                blinkin.setPixel2(Blinkin.PixelColor.YELLOW);
            }
            if(impGamepad1.x.isInitialPress()){
                blinkin.setPixel2(Blinkin.PixelColor.WHITE);
            }
        }
        blinkin.update();
    }
}
