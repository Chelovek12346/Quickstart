package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Movement {

    // Motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // PID
    private double drivekP, drivekI, kD, kF, shootkP, shootkI;
    private double integral = 0;
    private double lastError = 0;

    // Config
    private final double rotateSpeedDegPerSec;
    private double drivePowerScale;
    private double targetHeading;

    // Правый стик: насколько сильно его нужно отклонить, чтобы выбрать новое направление
    public static double SNAP_DEADZONE = 0.5;
    // Поворот всей «розы» направлений. Если робот смотрит не туда, куда стик —
    // поставь 90, 180 или -90
    public static double SNAP_OFFSET_DEG = 0;

    // Timing
    private long lastTime = System.nanoTime();

    public Movement(
            HardwareMap hardwareMap,
            double drivekP, double drivekI, double shootkP, double shootkI, double kD, double kF,
            double rotateSpeedDegPerSec,
            double drivePowerScale,
            double heading
    ) {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight  = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        this.drivekP = drivekP;
        this.drivekI = drivekI;
        this.shootkP = shootkP;
        this.shootkI = shootkI;
        this.kD = kD;
        this.kF = kF;
        this.rotateSpeedDegPerSec = rotateSpeedDegPerSec;
        this.drivePowerScale = drivePowerScale;

        targetHeading = normalizeAngle(heading);
    }

    public void setPowerAll(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    public void setPowerLeftRight(double powerL,double powerR) {
        frontLeft.setPower(powerL);
        frontRight.setPower(powerR);
        backLeft.setPower(powerL);
        backRight.setPower(powerR);
    }

    /**
     * @param x, y             движение в системе робота (как раньше)
     * @param stickRight       правый стик вправо  (gamepad.right_stick_x)
     * @param stickUp          правый стик вверх   (-gamepad.right_stick_y)
     * @param heading          текущий угол робота, градусы
     * @param shooting         true — автонаводка на calculatedTarget (любой угол)
     * @param calculatedTarget угол на цель, градусы
     */
    public void drive(double x, double y, double stickRight, double stickUp,
                      double heading, boolean shooting, double calculatedTarget, double headingTrim) {
        heading = normalizeAngle(heading);
        double deltaTime = getDeltaTimeSec();

        double kP, kI;
        double newTarget = targetHeading;

        if (shooting) {
            // Автонаводка — точный угол, без округления до 90°
            newTarget = calculatedTarget;
            kP = shootkP;
            kI = shootkI;
        } else {
            // Стик отклонён достаточно — выбираем одно из 4 направлений.
            // Диагонали делят стик на 4 сектора: это то же самое,
            // что округлить угол стика до ближайших 90°.
            if (Math.hypot(stickRight, stickUp) > SNAP_DEADZONE) {
                newTarget = snapTo90(stickAngleToField(stickRight, stickUp))+headingTrim;
            }
            // Стик отпущен — держим последний угол
            kP = drivekP;
            kI = drivekI;
        }

        newTarget = normalizeAngle(newTarget);

        // Цель сильно сменилась — сбрасываем интеграл, чтобы не тащить старую ошибку
        if (Math.abs(normalizeAngle(newTarget - targetHeading)) > 5) {
            integral = 0;
        }
        targetHeading = newTarget;

        double error = normalizeAngle(heading - targetHeading);
        integral = Math.max(-0.3, Math.min(0.3, integral * 0.9 + error * deltaTime));
        double derivative = (error - lastError) / deltaTime;

        double rx = kP * error
                + kI * integral
                + kD * derivative;

        lastError = error;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower  = (y + x + rx) / denominator * drivePowerScale;
        double backLeftPower   = (y - x + rx) / denominator * drivePowerScale;
        double frontRightPower = (y - x - rx) / denominator * drivePowerScale;
        double backRightPower  = (y + x - rx) / denominator * drivePowerScale;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    /**
     * Направление стика -> угол на поле.
     * Использует ту же систему координат, что и field-centric езда в Teleop:
     * стик вверх = поле -Y, стик вправо = поле -X.
     */
    private double stickAngleToField(double stickRight, double stickUp) {
        double fieldX = -stickRight;
        double fieldY = -stickUp;
        return Math.toDegrees(Math.atan2(fieldY, fieldX)) + SNAP_OFFSET_DEG;
    }

    /** Округление до ближайшего из 0, 90, 180, 270 (-90) градусов. */
    private double snapTo90(double angleDeg) {
        return normalizeAngle(Math.round(angleDeg / 90.0) * 90.0);
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    public void align(double heading, double calculatedTarget) {
        heading = normalizeAngle(heading);
        double deltaTime = getDeltaTimeSec();

        targetHeading = normalizeAngle(calculatedTarget);

        double error = normalizeAngle(heading - targetHeading);
        integral = Math.max(-0.3, Math.min(0.3, integral * 0.9 + error * deltaTime));
        double derivative = (error - lastError) / deltaTime;

        double rx = shootkP * error
                + shootkI * integral
                + kD * derivative
                + kF * (error > 0 ? 1 : -1);

        lastError = error;

        double denominator = Math.max(Math.abs(rx), 1);

        if (Math.abs(error) < 0.5) {
            rx = 0;
        }

        frontLeft.setPower(rx / denominator * drivePowerScale);
        backLeft.setPower(rx / denominator * drivePowerScale);
        frontRight.setPower(-rx / denominator * drivePowerScale);
        backRight.setPower(-rx / denominator * drivePowerScale);
    }

    public void setDrivePowerScale(double scale) {
        drivePowerScale = scale;
    }

    public double normalizeAngle(double angle) {
        angle = (angle + 180) % 360;
        if (angle < 0) angle += 360;
        return angle - 180;
    }

    private double getDeltaTimeSec() {
        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;
        return Math.max(dt, 1e-3); // защита от деления на ноль
    }
}