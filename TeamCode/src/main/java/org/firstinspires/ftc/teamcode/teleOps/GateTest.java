package org.firstinspires.ftc.teamcode.teleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Gate Test")
public class GateTest extends LinearOpMode {

    // Имя сервы из конфигурации на Driver Station
    static final String SERVO_NAME = "gate";

    // Начальная позициял
    static final double START_POSITION = 0.5;

    // Шаг изменения: вверх/вниз - крупный, влево/вправо - точный
    static final double BIG_STEP = 0.05;
    static final double SMALL_STEP = 0.01;

    @Override
    public void runOpMode() {
        Servo gate = hardwareMap.get(Servo.class, SERVO_NAME);

        double position = START_POSITION;

        telemetry.addLine("Нажми START - серва встанет на 0.5");
        telemetry.update();

        waitForStart();

        gate.setPosition(position);

        while (opModeIsActive()) {
            if (gamepad1.dpadUpWasPressed())    position += BIG_STEP;
            if (gamepad1.dpadDownWasPressed())  position -= BIG_STEP;
            if (gamepad1.dpadRightWasPressed()) position += SMALL_STEP;
            if (gamepad1.dpadLeftWasPressed())  position -= SMALL_STEP;

            // Позиция сервы должна быть от 0 до 1
            position = Range.clip(position, 0.0, 1.0);
            gate.setPosition(position);

            telemetry.addData("Позиция гейта", "%.2f", position);
            telemetry.addLine("Вверх/вниз: ±" + BIG_STEP + "   Вправо/влево: ±" + SMALL_STEP);
            telemetry.update();
        }
    }
}
