package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Commands {
    public DcMotorEx flywheelLeft;   // на нём стоит энкодер
    public DcMotorEx flywheelRight;
    private DcMotor intake;
    private DcMotor transfer;

    public Servo ramp, gate, hood;

    public double flyWheelRatio = 0.0;
    private final VoltageSensor battery;

    // ---------------- Регулятор маховика ----------------
    public static double TARGET_RPM = 1500;      // обороты маховика для startFlyWheel
    public static double TICKS_PER_REV = 28;     // ПРОВЕРЬ: проверни маховик на 10 оборотов и раздели getCurrentPosition() на 10
    public static double ENCODER_SIGN = 1;       // поставь -1, если при положительной мощности скорость отрицательная
    public static double FLY_KV = 0.00037;       // мощность на 1 тик/с (старт: 1 / макс. тиков/с)
    public static double FLY_KP = 0.0006;        // подними, пока не появятся колебания, потом убавь на 30%
    public static double FLY_KS = 0.067;           // стартовая мощность (обычно 0)
    public static double FLY_ALPHA = 0.25;       // сглаживание скорости (0..1, меньше = плавнее)
    public static double NOMINAL_VOLTAGE = 13.0;
    public static double AT_SPEED_TOL = 0.03;    // допуск ±3%

    private double flyTargetTicks = 0;           // тики/с
    private double flyVel = 0;                   // сглаженная скорость, тики/с
    private boolean flyVelInit = false;

    private double hoodPose = 0;

    public Commands(HardwareMap hardwareMap) {
        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");

        ramp = hardwareMap.get(Servo.class, "ramp");
        gate = hardwareMap.get(Servo.class, "gate");
        hood = hardwareMap.get(Servo.class, "hood");

        battery = hardwareMap.voltageSensor.iterator().next();

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelRight.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setHoodPose(double pose) { hood.setPosition(pose); }

    // ---------------- Маховик ----------------

    /** Ручная мощность (без регулятора). Выключает регулятор. */
    public void setFlyWheelVelocity(double power) {
        flyTargetTicks = 0;
        flywheelLeft.setPower(power);
        flywheelRight.setPower(power);
    }

    /** Задать целевые обороты маховика (об/мин). 0 = остановить. */
    public void setFlywheelRpm(double rpm) {
        if (rpm <= 0) {
            flyTargetTicks = 0;
            flywheelLeft.setPower(0);
            flywheelRight.setPower(0);
            return;
        }
        flyTargetTicks = rpm / 60.0 * TICKS_PER_REV;
    }

    /** ВЫЗЫВАТЬ В КАЖДОМ ЦИКЛЕ OpMode (teleop и auto). */
    public void updateFlywheel() {
        if (flyTargetTicks <= 0) return;   // регулятор выключен, моторы не трогаем

        double raw = ENCODER_SIGN * flywheelLeft.getVelocity();   // тики/с
        if (!flyVelInit) { flyVel = raw; flyVelInit = true; }
        else flyVel += FLY_ALPHA * (raw - flyVel);

        double ff = FLY_KS + FLY_KV * flyTargetTicks;
        double pid = FLY_KP * (flyTargetTicks - flyVel);

        double volt = battery.getVoltage();
        double scale = volt > 1.0 ? NOMINAL_VOLTAGE / volt : 1.0;   // компенсация просадки батареи

        double power = Math.max(0, Math.min(1, (ff + pid) * scale));
        flywheelLeft.setPower(power);
        flywheelRight.setPower(power);
    }

    public boolean flywheelAtSpeed() {
        return flyTargetTicks > 0 && Math.abs(flyTargetTicks - flyVel) < AT_SPEED_TOL * flyTargetTicks;
    }

    public double getFlywheelRpm() { return flyVel / TICKS_PER_REV * 60.0; }
    public double getFlywheelTargetRpm() { return flyTargetTicks / TICKS_PER_REV * 60.0; }

    // ---------------- Баллистика / колпак ----------------
    public static double MU = 0.15;           // подбирается на поле
    static final double G = 9.81;
    double gEff() { return G * (1 + MU); }

    // d, dh в метрах, v в м/с; возвращает радианы или NaN
    double getAngle(double d, double dh, double v) {
        double g = gEff();
        double disc = v*v*v*v - g*(g*d*d + 2*dh*v*v);
        if (disc < 0) return Double.NaN;
        double root = Math.sqrt(disc);
        double low  = Math.atan((v*v - root) / (g*d));
        double high = Math.atan((v*v + root) / (g*d));
        double MIN = Math.toRadians(40), MAX = Math.toRadians(85);
        if (low >= MIN && low <= MAX) return low;
        if (high >= MIN && high <= MAX) return high;
        return Double.NaN;
    }

    public void setHood(double d, double dh, double v) {
        double angRad = getAngle(d, dh, v);
        if (Double.isNaN(angRad)) return;   // не долетает / вне диапазона: серву не трогаем
        double ang = Math.toDegrees(angRad);
        double t = (ang - Config.hoodUpAngle) / (Config.hoodDownAngle - Config.hoodUpAngle);
        double lo = Math.min(Config.hoodUpPose, Config.hoodDownPose);
        double hi = Math.max(Config.hoodUpPose, Config.hoodDownPose);
        double pos = Config.hoodUpPose + t * (Config.hoodDownPose - Config.hoodUpPose);
        hood.setPosition(Math.max(lo, Math.min(hi, pos)));
    }

    public void setHood() { setHood(0.4, 1.3, 10); }   // тестовые числа, как было

    public Command closeGate = instant(() -> gate.setPosition(Config.gateClosePose));
    public Command openGate = instant(() -> gate.setPosition(Config.gateOpenPose));
    public Command raiseRamp = instant(() -> ramp.setPosition(Config.rampUpPose));
    public Command lowerRamp = instant(() -> ramp.setPosition(Config.rampDownPose));
    public Command raiseHood = instant(() -> hood.setPosition(Config.hoodUpPose));
    public Command lowerHood = instant(() -> hood.setPosition(Config.hoodDownPose));
    public Command startIntake = instant(() -> intake.setPower(1.0));
    public Command stopIntake = instant(() -> intake.setPower(0.0));
    public Command reverseIntake = instant(() -> intake.setPower(-1.0));
    public Command startTransfer = instant(() -> transfer.setPower(1.0));
    public Command stopTransfer = instant(() -> transfer.setPower(0.0));
    public Command reverseTransfer = instant(() -> transfer.setPower(-1.0));
    public Command autoHoodAngle = instant(() -> setHood());

    public Command startFlyWheel = instant(() -> setFlywheelRpm(TARGET_RPM));
    public Command stopFlyWheel = instant(() -> setFlywheelRpm(0));
}