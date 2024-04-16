package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Color;

import androidx.core.graphics.ColorUtils;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

    List<InputMechanism<?>> InputMechanisms;

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
                improvedGamepad1.left_trigger,
                improvedGamepad1.right_trigger,
                improvedGamepad1.back,
                improvedGamepad1.guide,
                improvedGamepad1.start
        };

        this.InputMechanisms = Arrays.asList(InputMechanisms);
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

    @Override

    public void loop() {
        improvedGamepad1.update();

        telemetry.addLine("left joy | ")
                .addData("x", "%.2f", improvedGamepad1.left_stick.x.getValue())
                .addData("y", "%.2f", improvedGamepad1.left_stick.y.getValue())
                .addData("t", "%.0f", improvedGamepad1.left_stick.angle.getValue())
                .addData("r", "%.2f", improvedGamepad1.left_stick.radius.getValue());

        telemetry.addLine("right joy | ")
                .addData("x", "%.2f", improvedGamepad1.right_stick.x.getValue())
                .addData("y", "%.2f", improvedGamepad1.right_stick.y.getValue())
                .addData("t", "%.0f", improvedGamepad1.right_stick.angle.getValue())
                .addData("r", "%.2f", improvedGamepad1.right_stick.radius.getValue());

        for (InputMechanism<?> button : InputMechanisms) {
            if (button.isPressed()) {
                telemetry.addData(button.getName(), "%s - %.2f", button.getValue(), button.getPressedElapseTime());
            }
        }

        if (improvedGamepad1.left_trigger.isPressed() || improvedGamepad1.right_trigger.isPressed()) {
            gamepad1.rumble(improvedGamepad1.left_trigger.getValue(), improvedGamepad1.right_trigger.getValue(), Gamepad.RUMBLE_DURATION_CONTINUOUS);
            isContRumble = true;
        } else if (isContRumble) {
            gamepad1.stopRumble();
            isContRumble = false;
        }

        float hue = improvedGamepad1.left_stick.angle.getValue().floatValue();
        int color = ColorUtils.HSLToColor(new float[]{hue, 1, 1});
        gamepad1.setLedColor(Color.red(color), Color.green(color), Color.blue(color), LED_DURATION_CONTINUOUS);

        telemetry.update();
    }
}
