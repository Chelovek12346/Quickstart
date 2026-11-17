package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Movement{

    // Motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Pedro localizer

    // PID
    private double drivekP, drivekI, kD, kF, shootkP, shootkI;
    private double integral = 0;
    private double lastError = 0;

    private boolean allianceBlue;

    // Config
    private final double rotateSpeedDegPerSec;
    private double drivePowerScale;
    private double targetHeading;

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

        targetHeading = heading;
    }
    public void setPowerAll(double power){
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }
    public void drive(double x, double y, double rotateStick, double heading, boolean shooting, double calculatedTarget) {
        heading = normalizeAngle(heading);

        double deltaTime = getDeltaTimeSec();

        double kP, kI;

        if (shooting) {
            targetHeading = calculatedTarget;
            kP = shootkP;
            kI = shootkI;
        } else {
            kP = drivekP;
            kI = drivekI;
        }

        targetHeading += rotateStick * rotateSpeedDegPerSec * deltaTime;
        targetHeading = normalizeAngle(targetHeading);

        double error = normalizeAngle(heading - targetHeading);
        integral = Math.max(-0.3, Math.min(0.3, integral * 0.9 + error * deltaTime));
        double derivative = (error - lastError) / deltaTime;

        double rx = kP * error
                + kI * integral
                + kD * derivative
                - kF * rotateStick;

        lastError = error;

        double denominator = Math.max(
                Math.abs(y) + Math.abs(x) + Math.abs(rx),
                1
        );

        double frontLeftPower  = (y + x + rx) / denominator * drivePowerScale;
        double backLeftPower   = (y - x + rx) / denominator * drivePowerScale;
        double frontRightPower = (y - x - rx) / denominator * drivePowerScale;
        double backRightPower  = (y + x - rx) / denominator * drivePowerScale;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public void align(double heading, double calculatedTarget) {
        heading = normalizeAngle(heading);

        double deltaTime = getDeltaTimeSec();

        double kP, kI;

        targetHeading = calculatedTarget;
        kP = shootkP;
        kI = shootkI;

        targetHeading = normalizeAngle(targetHeading);

        double error = normalizeAngle(heading - targetHeading);
        integral = Math.max(-0.3, Math.min(0.3, integral * 0.9 + error * deltaTime));
        double derivative = (error - lastError) / deltaTime;

        double rx = kP * error
                + kI * integral
                + kD * derivative
                + kF * (error > 0 ? 1 : -1);

        lastError = error;

        double denominator = Math.max(Math.abs(rx), 1);

        if (Math.abs(error) < 0.5) {
            rx = 0;
        }

        double frontLeftPower  = rx / denominator * drivePowerScale;
        double backLeftPower   = rx / denominator * drivePowerScale;
        double frontRightPower = -rx / denominator * drivePowerScale;
        double backRightPower  = -rx / denominator * drivePowerScale;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
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
        return dt;
    }


}
