package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;

public class Commands {
    public DcMotorEx flywheelLeft;   // на нём стоит энкодер
    public DcMotorEx flywheelRight;
    private DcMotor intake;
    private DcMotor transfer;

    public Servo ramp, gate, hood;

    public double flyWheelRatio = 0.0;
    private final VoltageSensor battery;

    // ---------------- Регулятор маховика ----------------
    public double targetRpm = 0;      // обороты маховика для startFlyWheel
    public static double TICKS_PER_REV = 28;     // ПРОВЕРЬ: проверни маховик на 10 оборотов и раздели getCurrentPosition() на 10
    public static double ENCODER_SIGN = 1;       // поставь -1, если при положительной мощности скорость отрицательная
    public static double FLY_KV = 0.0021;       // мощность на 1 тик/с (старт: 1 / макс. тиков/с)
    public static double FLY_KP = 0.004;        // подними, пока не появятся колебания, потом убавь на 30%
    public static double FLY_KS = 1.3721;           // стартовая мощность (обычно 0)
    public static double FLY_ALPHA = 0.25;       // сглаживание скорости (0..1, меньше = плавнее)
    public static double NOMINAL_VOLTAGE = 13.0;
    public static double AT_SPEED_TOL = 0.07;    // допуск ±3%

    public static double GOAL_H = 50.89;
    public static double PEAK_DISTANCE = 0;
    public static double BALL_CONSTANT = 625;
    public double FLOWER_INTAKE_POWER = 0.3;

    public double realRPM = 0;
    private double flyVel = 0;                   // сглаженная скорость, тики/с
    private boolean flyVelInit = false;
    private double hoodAngle = 0;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose leftHive = poseFactory.of(144-53, 86, 0);
    private final Pose rightHive = poseFactory.of(53, 86, 0);
    double k = (Config.hoodDownPose - Config.hoodUpPose) / (Config.hoodDownAngle - Config.hoodUpAngle);
    double b = Config.hoodDownPose - Config.hoodDownAngle * k;
    Telemetry telemetry;

    boolean flyWheelState, isLeftHive;

    public double TPS2RPM(double TPS){
        return TPS * 60.0 / TICKS_PER_REV;
    }

    public Commands(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
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

        flyWheelState = false;
        isLeftHive = true;
    }

    public void setHoodPose(double angle) {
        double t = k * angle + b;
        hood.setPosition(Math.min(Math.max(t, 0.32) , 0.96));
    }

    /** ВЫЗЫВАТЬ В КАЖДОМ ЦИКЛЕ OpMode (teleop и auto). */
    public void updateFlywheel() {
        if (!flyWheelState) {
            flywheelLeft.setPower(0);
            flywheelRight.setPower(0);
            return;
        }

        realRPM = ENCODER_SIGN * TPS2RPM(flywheelLeft.getVelocity());

        double ff = FLY_KS + FLY_KV * targetRpm;
        double pid = FLY_KP * (targetRpm - realRPM);

        telemetry.addData("ff", ff);
        telemetry.addData("pid", pid);

        double volt = battery.getVoltage();

        double power = Math.max(0, Math.min(1, (ff + pid) / volt));
        flywheelLeft.setPower(power);
        flywheelRight.setPower(power);
    }

    public boolean flywheelAtSpeed() {
//        return targetRpm > 0 && Math.abs(flyTargetTicks - flyVel) < AT_SPEED_TOL * flyTargetTicks;
        return targetRpm > 0;
    }
    public double getFlywheelRpm() { return flyVel / TICKS_PER_REV * 60.0; }
    public double getFlywheelTargetRpm() { return targetRpm; }

    public Command closeGate = instant(() -> gate.setPosition(Config.gateClosePose));
    public Command openGate = instant(() -> gate.setPosition(Config.gateOpenPose));
    public Command raiseRamp = instant(() -> ramp.setPosition(Config.rampUpPose));
    public Command lowerRamp = instant(() -> ramp.setPosition(Config.rampDownPose));
    public Command raiseHood = instant(() -> setHoodPose(Config.hoodUpAngle));
    public Command lowerHood = instant(() -> setHoodPose(Config.hoodDownAngle));
    public Command startIntake = instant(() -> intake.setPower(1.0));
    public Command stopIntake = instant(() -> intake.setPower(0.0));
    public Command reverseIntake = instant(() -> intake.setPower(-1.0));
    public Command startTransfer = instant(() -> transfer.setPower(1.0));
    public Command stopTransfer = instant(() -> transfer.setPower(0.0));
    public Command reverseTransfer = instant(() -> transfer.setPower(-1.0));
    public Command startFlyWheel = instant(() -> flyWheelState = true);
    public Command stopFlyWheel = instant(() -> flyWheelState = false);
    public Command setHood = instant(() -> setHoodPose(hoodAngle));
    public Command aimLeftHive = instant(() -> isLeftHive = true);
    public Command aimRightHive = instant(() -> isLeftHive = false);

    public double updateCalculations(Pose pose, double distanceTrimInches) {
        double virtualDist = 0;
        double iterVel = getFlywheelRpm();
        double iterAngle = hoodAngle;
        double iterTarget = 0;

        double robotX = pose.x();
        double robotY = pose.y();

        Pose hive = isLeftHive ? leftHive : rightHive;

        double distance = Math.hypot(robotX - hive.x(), robotY - hive.y()) - 5 + distanceTrimInches;

        iterAngle = Math.toDegrees(Math.atan(2 * GOAL_H / distance));

        double iterHeading = normalizeAngle(180 + Math.toDegrees(Math.atan2(hive.y() - robotY, hive.x() - robotX)));

        targetRpm = 12.542 * distance + 1902;
        hoodAngle = iterAngle;

        setHoodPose(hoodAngle);

        telemetry.addData("Distance", distance);
        telemetry.addData("Target RPM", targetRpm);
        telemetry.addData("RPM", realRPM);
        telemetry.addData("RPM diff", targetRpm-realRPM);

        return iterHeading;
    }

    public double normalizeAngle(double angle) {
        angle = (angle + 180) % 360;
        if (angle < 0) angle += 360;
        return angle - 180;
    }

    public boolean isLeftHive() {
        return isLeftHive;
    }
}