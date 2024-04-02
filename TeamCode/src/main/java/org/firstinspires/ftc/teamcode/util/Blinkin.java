package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class Blinkin {
    public RevBlinkinLedDriver blinkin;

    public enum PixelColor {
        GRAY,
        WHITE,
        YELLOW,
        GREEN
    }

    private int pixelColor1;
    private int pixelColor2;

    private long startTime;
    private int blinkTime = 500;
    private int restTime = 1000;

    public Blinkin(RevBlinkinLedDriver blinkin) {
        this.blinkin = blinkin;
        blinkin.resetDeviceConfigurationForOpMode();
        setPixel1(PixelColor.GRAY);
        setPixel2(PixelColor.GRAY);
        startTime = System.currentTimeMillis();
    }

    public void setBlinkTime(int blinkTime){
        this.blinkTime = blinkTime;
    }

    public void setRestTime(int restTime){
        this.restTime = restTime;
    }

    public void setPixel1(PixelColor color) {
        switch (color) {
            case GRAY: {
                pixelColor1 = 98;
                break;
            }
            case WHITE: {
                pixelColor1 = 97;
                break;
            }
            case YELLOW: {
                pixelColor1 = 85;
                break;
            }
            case GREEN: {
                pixelColor1 = 87;
                break;
            }
        }
    }

    public void setPixel2(PixelColor color) {
        switch (color) {
            case GRAY: {
                pixelColor2 = 98;
                break;
            }
            case WHITE: {
                pixelColor2 = 97;
                break;
            }
            case YELLOW: {
                pixelColor2 = 85;
                break;
            }
            case GREEN: {
                pixelColor2 = 87;
                break;
            }
        }
    }



    public void update() {
        long elapsedTime =( System.currentTimeMillis() - startTime) %(blinkTime * 2L + restTime);
        if(elapsedTime < blinkTime){
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(pixelColor1));
        }else if (elapsedTime < blinkTime * 2L){
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(pixelColor2));
        }else{
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }
    }
}
