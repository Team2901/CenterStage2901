package org.firstinspires.ftc.teamcode.Utilities;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.Executor;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

@Config
public class ImprovedGamepad {
    public static double StickDeadZone = 0.01;
    private Double lastKnownStickDeadZone = null;
    public static double TriggerDeadZone = 0.01;
    private Double lastKnownTriggerDeadZone = null;

    private final Gamepad hardwareGamepad;
    private final ElapsedTime timer;
    private final String name;

    public Joystick left_stick;
    public Joystick right_stick;

    public Button dpad_up;
    public Button dpad_down;
    public Button dpad_left;
    public Button dpad_right;

    public Button a;
    public Button b;
    public Button x;
    public Button y;

    public Button guide;
    public Button start;
    public Button back;

    public Button left_bumper;
    public Button right_bumper;
    public AnalogInput left_trigger;
    public AnalogInput right_trigger;

    Timer watchdogTimer = new Timer();

    public ImprovedGamepad(
            final Gamepad hardwareGamepad,
            final ElapsedTime timer,
            final String name) {

        this.hardwareGamepad = hardwareGamepad;
        this.timer = timer;
        this.name = (null != name) ? name : "";

        this.right_stick = new Joystick(String.format("%s_right_stick", this.name));
        this.left_stick = new Joystick(String.format("%s_left_stick", this.name));

        this.dpad_up = new Button(String.format("%s_dpad_up", this.name));
        this.dpad_down = new Button(String.format("%s_dpad_down", this.name));
        this.dpad_left = new Button(String.format("%s_dpad_left", this.name));
        this.dpad_right = new Button(String.format("%s_dpad_right", this.name));

        this.a = new Button(String.format("%s_a", this.name));
        this.b = new Button(String.format("%s_b", this.name));
        this.x = new Button(String.format("%s_x", this.name));
        this.y = new Button(String.format("%s_y", this.name));

        this.guide = new Button(String.format("%s_guide", this.name));
        this.start = new Button(String.format("%s_start", this.name));
        this.back = new Button(String.format("%s_back", this.name));

        this.left_bumper = new Button(String.format("%s_left_bumper", this.name));
        this.right_bumper = new Button(String.format("%s_right_bumper", this.name));
        this.left_trigger = new AnalogInput(String.format("%s_left_trigger", this.name));
        this.right_trigger = new AnalogInput(String.format("%s_right_trigger", this.name));

        //Thread t = new Thread(watchdog);
        //t.start();

        // If the opmode is TeleOp, set
        /*if (null != opmode.getClass().getAnnotation(TeleOp.class)) */ {
            watchdogTimer.schedule(new TimerTask() {
                @Override
                public void run() {
                    throw new RuntimeException("You forgot to call update!");
                }
            }, 10 * 1000L);
        }
    }

    public void update() {
        // We need to feed the watchdog
        //watchdog.feed();
        // If update is called, kill the timer
        // We don't really keep the watchdog timer going forever; assume if called once that
        // it will continue to be called
        /*if (watchdogTimer != null) {
            watchdogTimer.cancel();
            watchdogTimer = null;
        }*/

        // Check for Config updates
        // Having FTCDashboard Config only come from public static really messes with our class
        // abstraction design here.
        if ((lastKnownStickDeadZone == null) || (lastKnownStickDeadZone != StickDeadZone)) {
            left_stick.setDeadZone(StickDeadZone);
            right_stick.setDeadZone(StickDeadZone);
            lastKnownStickDeadZone = StickDeadZone;
        }
        if ((lastKnownTriggerDeadZone == null) || (lastKnownTriggerDeadZone != TriggerDeadZone)) {
            left_trigger.setDeadZone(TriggerDeadZone);
            right_trigger.setDeadZone(TriggerDeadZone);
            lastKnownTriggerDeadZone = TriggerDeadZone;
        }

        double time = timer.time();

        left_stick.update(Double.valueOf(hardwareGamepad.left_stick_x), Double.valueOf(hardwareGamepad.left_stick_y), hardwareGamepad.left_stick_button, time);
        right_stick.update(Double.valueOf(hardwareGamepad.right_stick_x), Double.valueOf(hardwareGamepad.right_stick_y), hardwareGamepad.right_stick_button, time);

        dpad_up.update(hardwareGamepad.dpad_up, time);
        dpad_down.update(hardwareGamepad.dpad_down, time);
        dpad_left.update(hardwareGamepad.dpad_left, time);
        dpad_right.update(hardwareGamepad.dpad_right, time);

        a.update(hardwareGamepad.a, time);
        b.update(hardwareGamepad.b, time);
        x.update(hardwareGamepad.x, time);
        y.update(hardwareGamepad.y, time);

        guide.update(hardwareGamepad.guide, time);
        start.update(hardwareGamepad.start, time);
        back.update(hardwareGamepad.back, time);

        left_bumper.update(hardwareGamepad.left_bumper, time);
        right_bumper.update(hardwareGamepad.right_bumper, time);
        left_trigger.update(Double.valueOf(hardwareGamepad.left_trigger), time);
        right_trigger.update(Double.valueOf(hardwareGamepad.right_trigger), time);
    }

    /*
    UpdateWatchdog watchdog = new UpdateWatchdog();
    private class UpdateWatchdog implements Runnable
    {
        BlockingQueue<Boolean> watchdogFood = new LinkedBlockingQueue<>();

        @Override
        public void run() {
            try {
                Boolean food = watchdogFood.poll(10, TimeUnit.SECONDS);
                if (food == null) {
                    throw new RuntimeException("You forgot to call update!");
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        public void feed() {
            watchdogFood.add(true);
        }
    }
     */
}
