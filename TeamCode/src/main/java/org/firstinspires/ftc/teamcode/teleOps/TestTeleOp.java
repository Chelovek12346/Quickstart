package org.firstinspires.ftc.teamcode.teleOps;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
@TeleOp(name = "Teleop")
public class TestTeleOp extends OpMode {

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private Follower follower;
    private Commands commands;
    private boolean flyWheelState, hoodState, rampState;
    private final Pose startPose = poseFactory.of(0, 0, 0);
    private double flyWheelRatio = 0.0;

    public static double kP = 0.01, kI = 0.01, kD = 0.001, kF = 0.45, rotateSpeedDegPerSec = 220, drivePowerScale=1;

    private Movement movement;

    private Command startFlyWheel2 = instant(() -> {
        commands.flywheelLeft.setPower(flyWheelRatio);
        commands.flywheelRight.setPower(flyWheelRatio);
    });

    private double gatePose, hoodPose, rampPose;
    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        movement = new Movement(
                hardwareMap,
                kP,
                kI,
                kD,
                kF,
                rotateSpeedDegPerSec,
                drivePowerScale,
                follower
        );

        commands = new Commands(hardwareMap);
        flyWheelState = false;
        hoodState = false;
        rampState = false;

        schedule(commands.closeGate);
        schedule(commands.raiseHood);
        schedule(commands.raiseRamp);

        Scheduler.execute();
    }
    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe  = gamepad1.left_stick_x;
        double rotate  = -gamepad1.right_stick_x;

        if (Math.abs(forward) < 0.05) forward = 0;
        if (Math.abs(strafe)  < 0.05) strafe  = 0;
        if (Math.abs(rotate)  < 0.05) rotate  = 0;


        double heading = follower.pose().heading();
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);
        double xField = strafe * cos - forward * sin;
        double yField = strafe * sin + forward * cos;

        movement.drive(
                xField,
                yField,
                rotate
        );

        follower.update();

        if (gamepad1.rightTriggerWasPressed()) {
            schedule(commands.openGate);
            schedule(commands.startIntake);
            schedule(commands.startTransfer);
        }
        if (gamepad1.rightTriggerWasReleased()) {
            schedule(commands.closeGate);
            schedule(commands.stopIntake);
            schedule(commands.stopTransfer);
        }

        if (gamepad1.aWasPressed()) {
            if (flyWheelState) {
                schedule(commands.stopFlyWheel);
            } else {
                schedule(startFlyWheel2);
            }
            flyWheelState = !flyWheelState;
        }

        if (gamepad1.xWasPressed()) {
            if (hoodState) {
                schedule(commands.raiseHood);
            } else {
                schedule(commands.lowerHood);
            }
            hoodState = !hoodState;
        }

        if (gamepad1.bWasPressed()) {
            if (rampState) {
                schedule(commands.raiseRamp);
            } else {
                schedule(commands.lowerRamp);
            }
            rampState = !rampState;
        }

        if (gamepad1.leftTriggerWasPressed()) {
            schedule(commands.startIntake);
            schedule(commands.startTransfer);
        }
        if (gamepad1.leftTriggerWasReleased()) {
            schedule(commands.stopIntake);
            schedule(commands.stopTransfer);
        }
        if (gamepad1.yWasPressed()) {
            schedule(commands.autoHoodAngle);
        }
        if (gamepad1.dpadUpWasPressed()){
            flyWheelRatio += 0.01;
        }
        if (gamepad1.dpadDownWasPressed()){
            flyWheelRatio -= 0.01;
        }

        Scheduler.execute();

        telemetry.addData("Hood:", hoodPose);
        telemetry.addData("Ramp:", rampPose);
        telemetry.addData("Gate:", gatePose);
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading", follower.pose().heading());
        telemetry.addData("fly wheel", flyWheelRatio);
        telemetry.update();

    }
}