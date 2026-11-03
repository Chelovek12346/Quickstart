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
    private Follower follower;

    // PID
    private double kP, kI, kD, kF;
    private double integral = 0;
    private double lastError = 0;

    private boolean allianceBlue;

    // Config
    private double rotateSpeedDegPerSec;
    private double drivePowerScale;

    // Heading target (degrees)
    private double targetHeading;

    // Timing
    private long lastTime = System.nanoTime();

    public Movement(
            HardwareMap hardwareMap,
            double kP, double kI, double kD, double kF,
            double rotateSpeedDegPerSec,
            double drivePowerScale,
            Follower follower
    ) {

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight  = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        this.follower = follower;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.rotateSpeedDegPerSec = rotateSpeedDegPerSec;
        this.drivePowerScale = drivePowerScale;

        targetHeading = follower.pose().heading();

    }


    public void drive(double x, double y, double rotateStick) {
        Pose pose = follower.pose();
        double heading = Math.toDegrees(pose.heading());
        heading = normalizeAngle(heading);

        double deltaTime = getDeltaTimeSec();

        targetHeading += rotateStick * rotateSpeedDegPerSec * deltaTime;
        targetHeading = normalizeAngle(targetHeading);

        double error = normalizeAngle(heading - targetHeading);
        integral = integral * 0.9 + error * deltaTime;
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

    public void setTargetHeading(double targetDeg) {
        targetHeading = normalizeAngle(targetDeg);
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    public void setDrivePowerScale(double scale) {
        drivePowerScale = scale;
    }

    private double normalizeAngle(double angle) {
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
