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
    private final int BLACK = 100;
    private final int PINK = 78;

    private final int ALIGNED = PINK;
    private final int AUTO = PURPLE;
    private final int TELEOP = GREEN;


    public enum PixelColor {
        WHITE,
        YELLOW,
        GREEN,
        PURPLE
    }

    public enum PixelStatus {
        REQUESTED,//not used
        HELD,//not used
        EMPTY,//not used
        AUTO,
        TELEOP,
        ALIGNED,//to backdrop
    }

    private int pixelLeft = BLACK;
    private int pixelRight = BLACK;

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
    }

    public void setPixelLeft(PixelColor color) {
        startTime = System.currentTimeMillis();
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
        startTime = System.currentTimeMillis();
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

//    public void cycleLeftPixel() {
//        setPixelLeftStatus(PixelStatus.REQUESTED);
//        startTime = System.currentTimeMillis();
//        switch (pixelLeft) {
//            case WHITE: {
//                pixelLeft = YELLOW;
//                break;
//            }
//            case YELLOW: {
//                pixelLeft = GREEN;
//                break;
//            }
//            case GREEN: {
//                pixelLeft = PURPLE;
//                break;
//            }
//            case PURPLE: {
//                pixelLeft = BLACK;
//                break;
//            }
//            case BLACK: {
//                pixelLeft = WHITE;
//                break;
//            }
//        }
//    }
//
//    public void cycleRightPixel() {
//        setPixelRightStatus(PixelStatus.REQUESTED);
//        startTime = System.currentTimeMillis();
//        switch (pixelRight) {
//            case WHITE: {
//                pixelRight = YELLOW;
//                break;
//            }
//            case YELLOW: {
//                pixelRight = GREEN;
//                break;
//            }
//            case GREEN: {
//                pixelRight = PURPLE;
//                break;
//            }
//            case PURPLE: {
//                pixelRight = BLACK;
//                break;
//            }
//            case BLACK: {
//                pixelRight = WHITE;
//                break;
//            }
//        }
//    }

    public void setPixelStatus(PixelStatus status) {
        setPixelLeftStatus(status);
        setPixelRightStatus(status);
    }

    public void setPixelLeftStatus(PixelStatus status) {
        startTime = System.currentTimeMillis();
        pixelLeftStatus = status;
//        if (status == PixelStatus.EMPTY) {
//            blinkinLeft.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(EMPTY_PATTERN));
//        }
    }

    public void setPixelRightStatus(PixelStatus status) {
        startTime = System.currentTimeMillis();
        pixelRightStatus = status;
//        if (status == PixelStatus.EMPTY) {
//            blinkinRight.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(EMPTY_PATTERN));
//        }
    }

    long elapsedTime;

    private void colorSide(boolean left) {
        RevBlinkinLedDriver blinkinModule = left ? blinkinLeft : blinkinRight;
        PixelStatus moduleStatus = left ? pixelLeftStatus : pixelRightStatus;
        int moduleColor = left ? pixelLeft : pixelRight;

        switch (moduleStatus) {
            case HELD: {
                if (elapsedTime < blinkPeriod) {
                    blinkinModule.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(moduleColor));
                } else {
                    blinkinModule.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                }
                break;
            }
            case REQUESTED: {
                blinkinModule.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(moduleColor));
                break;
            }
            case EMPTY: {
                blinkinModule.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(EMPTY_PATTERN));
                break;
            }
            case AUTO: {
                blinkinModule.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(AUTO));
                break;
            }
            case TELEOP: {
                blinkinModule.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(TELEOP));
                break;
            }
            case ALIGNED: {
                blinkinModule.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(ALIGNED));
                break;
            }
        }
    }

    public void update() {
        elapsedTime = (System.currentTimeMillis() - startTime) % (blinkPeriod * 2L);

        colorSide(true);
        colorSide(false);
    }

    public void update(int color) {
        blinkinLeft.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(color));
        blinkinRight.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(color));
    }

    public PixelStatus getPixelLeftStatus() {
        return pixelLeftStatus;
    }

    public PixelStatus getPixelRightStatus() {
        return pixelRightStatus;
    }
}
