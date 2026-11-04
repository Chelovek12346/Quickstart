package org.firstinspires.ftc.teamcode.teleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Commands;

/**
 * Подбор kS и kV маховика по напряжению.
 *
 * Управление (gamepad1):
 *   dpad_up / dpad_down      напряжение +step / -step
 *   right_bumper / left_bumper   сменить шаг (0.05 -> 0.1 -> 0.25 -> 0.5 В)
 *   A   включить подачу напряжения
 *   B   выключить и обнулить напряжение (и сбросить найденный kS)
 *   X   зафиксировать kS вручную (текущее напряжение)
 *   Y   сбросить авто-определение kS (начать подъём заново)
 *
 * Как найти kS: включи A, медленно поднимай напряжение с шагом 0.05 В.
 * Как только маховик впервые стронулся, напряжение запоминается как kS (показано в телеметрии).
 *
 * Как найти kV: подними напряжение до рабочего (чтобы обороты были около целевых),
 * дождись установившейся скорости. Телеметрия сама посчитает значения для Commands:
 *   FLY_KS = kS / 12,   FLY_KV = (V - kS) / 12 / (тик/с)
 */
@TeleOp(name = "Voltage Test")
public class FlywheelVoltageTest extends LinearOpMode {

    static final double[] STEPS = {0.05, 0.1, 0.25, 0.5};
    static final double MOVE_THRESHOLD = 10;   // тик/с: порог "маховик тронулся"
    static final double NOMINAL = 12.0;

    @Override
    public void runOpMode() {
        Commands commands = new Commands(hardwareMap);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        double volts = 0;
        boolean on = false;
        int stepIdx = 1;
        double kS = Double.NaN;


        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpadUpWasPressed()) volts += STEPS[stepIdx];
            if (gamepad1.dpadDownWasPressed()) volts -= STEPS[stepIdx];
            if (gamepad1.rightBumperWasPressed()) stepIdx = Math.min(STEPS.length - 1, stepIdx + 1);
            if (gamepad1.leftBumperWasPressed()) stepIdx = Math.max(0, stepIdx - 1);


            if (gamepad1.a) on = true;
            if (gamepad1.b) { on = false; volts = 0; kS = Double.NaN; }
            if (gamepad1.y) kS = Double.NaN;
            if (gamepad1.x && on) kS = volts;

            double batt = battery.getVoltage();
            volts = Math.max(0, Math.min(batt, volts));

            double power = on ? Math.max(0, Math.min(1, volts / batt)) : 0;
            commands.flywheelLeft.setPower(power);
            commands.flywheelRight.setPower(power);

            double vel = Commands.ENCODER_SIGN * commands.flywheelLeft.getVelocity();   // тик/с
            double rpm = vel / Commands.TICKS_PER_REV * 60.0;

            // авто-определение kS: первое напряжение, при котором маховик тронулся
            if (on && Double.isNaN(kS) && Math.abs(vel) > MOVE_THRESHOLD) kS = volts;

            telemetry.addData("ON", on);
            telemetry.addData("Задано, В", "%.2f  (шаг %.2f)", volts, STEPS[stepIdx]);
            telemetry.addData("Батарея, В", "%.2f", batt);
            telemetry.addData("Мощность", "%.3f", power);
            telemetry.addData("Скорость, тик/с", "%.0f", vel);
            telemetry.addData("Скорость, об/мин", "%.0f", rpm);
            telemetry.addData("Позиция, тики", commands.flywheelLeft.getCurrentPosition());
            telemetry.addLine("-------- результат --------");
            telemetry.addData("kS, В", Double.isNaN(kS) ? "не найден" : String.format("%.2f", kS));
            if (!Double.isNaN(kS)) {
                telemetry.addData("-> FLY_KS", "%.4f", kS / NOMINAL);
                if (vel > 100 && volts > kS) {
                    telemetry.addData("-> FLY_KV (по текущей точке)", "%.6f", (volts - kS) / NOMINAL / vel);
                }
            }
            telemetry.update();
        }

        commands.flywheelLeft.setPower(0);
        commands.flywheelRight.setPower(0);
    }
}