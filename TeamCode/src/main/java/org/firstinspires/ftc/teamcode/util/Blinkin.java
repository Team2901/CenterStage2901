package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class Blinkin {
    public RevBlinkinLedDriver blinkinLeft;
    public RevBlinkinLedDriver blinkinRight;

    private final int EMPTY_PATTERN = 29;

    //must be > 1000ms for epilepsy
    private final int blinkPeriod = 1100;

    private final int WHITE = 97;
    private final int YELLOW = 85;
    private final int GREEN = 87;
    private final int PURPLE = 95;


    public enum PixelColor {
        WHITE,
        YELLOW,
        GREEN,
        PURPLE
    }

    public enum PixelStatus {
        REQUESTED,
        HELD,
        EMPTY
    }

    private int pixelLeft;
    private int pixelRight;

    private PixelStatus pixelLeftStatus;
    private PixelStatus pixelRightStatus;

    private long startTime;

    public Blinkin(RevBlinkinLedDriver blinkinLeft, RevBlinkinLedDriver blinkinRight) {
        this.blinkinLeft = blinkinLeft;
        this.blinkinRight = blinkinRight;

        blinkinLeft.resetDeviceConfigurationForOpMode();
        blinkinRight.resetDeviceConfigurationForOpMode();
        setPixelStatus(PixelStatus.EMPTY);

        startTime = System.currentTimeMillis();
//        setPixelLeft(PixelColor.GRAY);
//        setPixelRight(PixelColor.GRAY);
    }

//    public void setBlinkTime(int blinkTime) {
//        this.blinkTime = blinkTime;
//    }
//
//    public void setRestTime(int restTime) {
//        this.restTime = restTime;
//    }

    public void setPixelLeft(PixelColor color) {
        switch (color) {
            case WHITE: {
                pixelLeft = WHITE;
                break;
            }
            case YELLOW: {
                pixelLeft = YELLOW;
                break;
            }
            case GREEN: {
                pixelLeft = GREEN;
                break;
            }
            case PURPLE: {
                pixelLeft = PURPLE;
                break;
            }
        }
    }

    public void setPixelRight(PixelColor color) {
        switch (color) {
            case WHITE: {
                pixelRight = WHITE;
                break;
            }
            case YELLOW: {
                pixelRight = YELLOW;
                break;
            }
            case GREEN: {
                pixelRight = GREEN;
                break;
            }
            case PURPLE: {
                pixelRight = PURPLE;
                break;
            }
        }
    }

    public void setPixelStatus(PixelStatus status){
        setPixelLeftStatus(status);
        setPixelRightStatus(status);
    }

    public void setPixelLeftStatus(PixelStatus status) {
        pixelLeftStatus = status;
//        if (status == PixelStatus.EMPTY) {
//            blinkinLeft.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(EMPTY_PATTERN));
//        }
    }

    public void setPixelRightStatus(PixelStatus status) {
        pixelRightStatus = status;
//        if (status == PixelStatus.EMPTY) {
//            blinkinRight.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(EMPTY_PATTERN));
//        }
    }

    long elapsedTime;

    private void colorSide(boolean left){
        RevBlinkinLedDriver blinkinModule = left?blinkinLeft:blinkinRight;
        PixelStatus moduleStatus = left?pixelLeftStatus:pixelRightStatus;
        int moduleColor = left?pixelLeft:pixelRight;

        switch (moduleStatus){
            case REQUESTED:{
                if(elapsedTime < blinkPeriod){
                    blinkinModule.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(moduleColor));
                }else{
                    blinkinModule.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                }
                break;
            }
            case HELD:{
                blinkinModule.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(moduleColor));
                break;
            }
            case EMPTY:{
                blinkinModule.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(EMPTY_PATTERN));
                break;
            }
        }
    }

    public void update() {
        elapsedTime =( System.currentTimeMillis() - startTime) %(blinkPeriod * 2L);

        colorSide(true);
        colorSide(false);
    }
}
