package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.instant;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Commands {
    public DcMotor flywheelLeft;
    public DcMotor flywheelRight;
    private DcMotor intake;
    private DcMotor transfer;

    public Servo ramp, gate, hood;

    private double hoodPose = 0;
    private double flyWheelVelocity = 0;

    public Commands(HardwareMap hardwareMap) {
        flywheelLeft = hardwareMap.get(DcMotor.class, "flywheelLeft");;
        flywheelRight = hardwareMap.get(DcMotor.class, "flywheelRight");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");

        ramp = hardwareMap.get(Servo.class, "ramp");
        gate = hardwareMap.get(Servo.class, "gate");
        hood = hardwareMap.get(Servo.class, "hood");


        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelRight.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setHoodPose(double pose) { hood.setPosition(pose); }
    public void setFlyWheelVelocity(double vel) {
        flywheelLeft.setPower(vel);
        flywheelRight.setPower(vel);
    }

    public static double MU = 0.15;           // подбирается на поле
    static final double G = 9.81;
    double gEff() { return G * (1 + MU); }

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

    public void setHood(){
        double ang = Math.toDegrees(getAngle(0.4, 1.3,10));
        double t = (ang - Config.hoodUpAngle) / (Config.hoodDownAngle - Config.hoodUpAngle);
        hood.setPosition(Math.max(Math.min(Config.hoodUpPose, Config.hoodDownPose),
                Math.min(Math.max(Config.hoodUpPose, Config.hoodDownPose),
                        Config.hoodUpPose + t * (Config.hoodDownPose - Config.hoodUpPose))));
    }


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

    public Command startFlyWheel = instant(() -> {
        flywheelLeft.setPower(0.7);
        flywheelRight.setPower(0.7);
    });
    public Command stopFlyWheel = instant(() -> {
        flywheelLeft.setPower(0);
        flywheelRight.setPower(0);
    });


}
