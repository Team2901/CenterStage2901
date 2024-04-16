package org.firstinspires.ftc.teamcode.TeleOp;

import android.annotation.SuppressLint;
import android.graphics.Color;

import androidx.core.graphics.ColorUtils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.Utilities.InputMechanism;

import java.util.Arrays;
import java.util.List;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;

@TeleOp(name = "Imp Gamepad Tester", group = "Utilities")
public class ImprovedGamepadTester extends OpMode {

    ImprovedGamepad improvedGamepad1;

    List<InputMechanism<?>> Buttons;

    boolean isContRumble = false;
    boolean useRawValues = false;

    @Override
    public void init() {
        improvedGamepad1 = new ImprovedGamepad(this.gamepad1, new ElapsedTime(), "", this.telemetry);

        InputMechanism<?> [] InputMechanisms = {
                improvedGamepad1.a,
                improvedGamepad1.b,
                improvedGamepad1.x,
                improvedGamepad1.y,
                improvedGamepad1.dpad_up,
                improvedGamepad1.dpad_left,
                improvedGamepad1.dpad_down,
                improvedGamepad1.dpad_right,
                improvedGamepad1.left_bumper,
                improvedGamepad1.right_bumper,
                improvedGamepad1.back,
                improvedGamepad1.guide,
                improvedGamepad1.start
        };

        this.Buttons = Arrays.asList(InputMechanisms);
    }

    @Override
    public void init_loop() {
        super.init_loop();

        if (this.gamepad1.a) {
            useRawValues = true;
            improvedGamepad1.setStickDeadZone(0);
            improvedGamepad1.setTriggerDeadZone(0);
        } else if (this.gamepad1.b) {
            useRawValues = false;
            improvedGamepad1.setStickDeadZone(ImprovedGamepad.DEFAULT_STICK_DEAD_ZONE);
            improvedGamepad1.setTriggerDeadZone(ImprovedGamepad.DEFAULT_TRIGGER_DEAD_ZONE);
        }

        telemetry.addData("Raw Values", useRawValues);
        telemetry.update();
    }

    @SuppressLint("DefaultLocale")
    @Override

    public void loop() {
        improvedGamepad1.update();

        telemetry.addLine("left joy");
        telemetry.addLine(String.format("x=%.2f y=%.2f", improvedGamepad1.left_stick.x.getValue(), improvedGamepad1.left_stick.y.getValue()));
        telemetry.addLine(String.format("t=%.0f r=%.2f", improvedGamepad1.left_stick.angle.getValue(), improvedGamepad1.left_stick.radius.getValue()));

        telemetry.addLine("right joy");
        telemetry.addLine(String.format("x=%.2f y=%.2f", improvedGamepad1.right_stick.x.getValue(), improvedGamepad1.right_stick.y.getValue()));
        telemetry.addLine(String.format("t=%.0f r=%.2f", improvedGamepad1.right_stick.angle.getValue(), improvedGamepad1.right_stick.radius.getValue()));

        telemetry.addLine("Fingers");
        if(gamepad1.touchpad_finger_1) {
            telemetry.addLine(String.format("(1) x=%5.2f y=%5.2f", gamepad1.touchpad_finger_1_x, gamepad1.touchpad_finger_1_y));
        }

        if (gamepad1.touchpad_finger_2) {
            telemetry.addLine(String.format("(2) x=%5.2f y=%5.2f", gamepad1.touchpad_finger_2_x, gamepad1.touchpad_finger_2_y));
        }

        if (!gamepad1.touchpad_finger_1 && !gamepad1.touchpad_finger_2) {
            telemetry.addLine("no touchy");
        }

        if (improvedGamepad1.left_trigger.isPressed() || improvedGamepad1.right_trigger.isPressed()) {
            telemetry.addLine(String.format("(l) %.2f", improvedGamepad1.left_trigger.getValue()));
            telemetry.addLine(String.format("(r) %.2f", improvedGamepad1.right_trigger.getValue()));
            gamepad1.rumble(improvedGamepad1.left_trigger.getValue(), improvedGamepad1.right_trigger.getValue(), Gamepad.RUMBLE_DURATION_CONTINUOUS);
            isContRumble = true;
        } else if (isContRumble) {
            gamepad1.stopRumble();
            isContRumble = false;
        }

        for (InputMechanism<?> button : Buttons) {
            if (button.isPressed()) {
                telemetry.addLine(String.format("%s - %f", button.getName(), button.getPressedElapseTime()));
            }
        }

        float hue = improvedGamepad1.left_stick.angle.getValue().floatValue();
        int color = ColorUtils.HSLToColor(new float[]{hue, 1, 1});
        gamepad1.setLedColor(Color.red(color), Color.green(color), Color.blue(color), LED_DURATION_CONTINUOUS);

        telemetry.update();
    }
}
