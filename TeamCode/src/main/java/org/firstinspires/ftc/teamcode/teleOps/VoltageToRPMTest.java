package org.firstinspires.ftc.teamcode.teleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Тест маховика по напряжению.
 *
 * D-pad вверх / вниз     — напряжение +0.5 В / -0.5 В
 * D-pad вправо / влево   — напряжение +0.1 В / -0.1 В (точная подстройка)
 * X                      — стоп (0 В)
 */
@TeleOp(name = "52")
public class VoltageToRPMTest extends OpMode {

    // Тиков на оборот вала маховика. Мотор без редуктора (6000 RPM) = 28.
    // Если между мотором и маховиком есть передача — RPM маховика = RPM мотора * передаточное число.
    public static double TICKS_PER_REV = 28;
    public static double MAX_VOLTAGE = 14.0;

    // Имена должны совпадать с конфигурацией на Driver Hub
    private DcMotorEx fly1, fly2;
    private VoltageSensor battery;

    private double targetVoltage = 0;
    private boolean prevUp, prevDown, prevRight, prevLeft;

    private final ElapsedTime timer = new ElapsedTime();
    private double prevTime = 0;
    private int prevPos1 = 0, prevPos2 = 0;
    private double rpm1 = 0, rpm2 = 0;

    @Override
    public void init() {
        fly1 = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        fly2 = hardwareMap.get(DcMotorEx.class, "flywheelRight");

        // Если моторы стоят друг напротив друга на одном валу — один нужно развернуть
        fly1.setDirection(DcMotorSimple.Direction.FORWARD);
        fly2.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx m : new DcMotorEx[]{fly1, fly2}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // маховик свободно останавливается
        }

        battery = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void start() {
        timer.reset();
        prevTime = 0;
        prevPos1 = fly1.getCurrentPosition();
        prevPos2 = fly2.getCurrentPosition();
    }

    @Override
    public void loop() {
        // --- регулировка напряжения ---
        if (gamepad1.dpad_up && !prevUp) targetVoltage += 0.5;
        if (gamepad1.dpad_down && !prevDown) targetVoltage -= 0.5;
        if (gamepad1.dpad_right && !prevRight) targetVoltage += 0.1;
        if (gamepad1.dpad_left && !prevLeft) targetVoltage -= 0.1;
        if (gamepad1.x) targetVoltage = 0;
        prevUp = gamepad1.dpad_up;
        prevDown = gamepad1.dpad_down;
        prevRight = gamepad1.dpad_right;
        prevLeft = gamepad1.dpad_left;

        targetVoltage = Math.max(0, Math.min(MAX_VOLTAGE, targetVoltage));
        targetVoltage = Math.round(targetVoltage * 10) / 10.0;

        // --- напряжение -> мощность ---
        // setPower задаёт долю от напряжения аккумулятора,
        // поэтому делим нужное напряжение на текущее напряжение батареи
        double batteryVoltage = battery.getVoltage();
        double power = Math.min(1.0, targetVoltage / batteryVoltage);

        fly1.setPower(power);
        fly2.setPower(power);

        // --- скорость через позицию и dt, пересчёт раз в 100 мс ---
        double now = timer.seconds();
        double dt = now - prevTime;
        if (dt >= 0.5) {
            int pos1 = fly1.getCurrentPosition();
            rpm1 = (pos1 - prevPos1) / dt / TICKS_PER_REV * 60.0;
            prevPos1 = pos1;
            prevTime = now;
        }

        // --- телеметрия ---
        telemetry.addData("Заданное напряжение", "%.1f В", targetVoltage);
        telemetry.addData("Напряжение батареи", "%.2f В", batteryVoltage);
        telemetry.addData("Мощность (setPower)", "%.3f", power);
        if (targetVoltage > batteryVoltage) {
            telemetry.addLine("!! Заданное напряжение выше батареи — упёрлись в 1.0");
        }
        telemetry.addLine();
        telemetry.addData("Мотор 1 RPM", "%.0f", rpm1);

        telemetry.addLine();
        telemetry.addLine("D-pad ↑↓ ±0.5 В | ←→ ±0.1 В | X стоп");
        telemetry.update();
    }

    @Override
    public void stop() {
        fly1.setPower(0);
        fly2.setPower(0);
    }
}