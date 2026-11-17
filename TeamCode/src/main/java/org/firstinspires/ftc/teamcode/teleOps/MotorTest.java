package org.firstinspires.ftc.teamcode.teleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * D-pad вверх / вниз — мощность +0.1 / -0.1 (от -1 до 1)
 * X                  — мощность в 0
 */
@TeleOp(name = "67")
public class MotorTest extends OpMode {

    public static double TICKS_PER_REV = 384.5; // goBILDA 435 RPM

    // Имена должны совпадать с конфигурацией на Driver Hub
    private final String[] names = {"motor1", "motor2", "motor3", "motor4"};
    private final DcMotorEx[] motors = new DcMotorEx[4];

    private final int[] prevPos = new int[4];
    private final double[] rpm = new double[4];

    private final ElapsedTime timer = new ElapsedTime();
    private double prevTime = 0;

    private double power = 0;
    private boolean prevUp, prevDown;

    @Override
    public void init() {
        for (int i = 0; i < 4; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, names[i]);
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    @Override
    public void start() {
        timer.reset();
        prevTime = 0;
        for (int i = 0; i < 4; i++) prevPos[i] = motors[i].getCurrentPosition();
    }

    @Override
    public void loop() {
        // --- регулировка мощности ---
        if (gamepad1.dpad_up && !prevUp) power = Math.min(1.0, power + 0.1);
        if (gamepad1.dpad_down && !prevDown) power = Math.max(-1.0, power - 0.1);
        if (gamepad1.x) power = 0;
        prevUp = gamepad1.dpad_up;
        prevDown = gamepad1.dpad_down;

        power = Math.round(power * 10) / 10.0; // убираем 0.30000000004

        for (DcMotorEx m : motors) m.setPower(power);

        // --- RPM через позицию и dt, пересчёт раз в 100 мс ---
        double now = timer.seconds();
        double dt = now - prevTime;
        if (dt >= 1.0) {
            for (int i = 0; i < 4; i++) {
                int pos = motors[i].getCurrentPosition();
                double ticksPerSec = (pos - prevPos[i]) / dt;
                rpm[i] = ticksPerSec / TICKS_PER_REV * 60.0;
                prevPos[i] = pos;
            }
            prevTime = now;
        }

        // --- телеметрия ---
        telemetry.addData("Мощность", "%.1f", power);
        telemetry.addLine();
        for (int i = 0; i < 4; i++) {
            telemetry.addData(names[i] + " RPM", "%.0f", rpm[i]);
        }
        telemetry.addLine();
        telemetry.addLine("D-pad ↑↓ мощность ±0.1 | X стоп");
        telemetry.update();
    }

    @Override
    public void stop() {
        for (DcMotorEx m : motors) m.setPower(0);
    }
}