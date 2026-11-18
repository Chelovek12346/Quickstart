package org.firstinspires.ftc.teamcode.teleOps;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "1488")
public class TestT extends OpMode {

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private Follower follower;
    private Commands commands;
    private boolean flyWheelState, hoodState, rampState;
    private final Pose startPose = poseFactory.of(0, 0, 0);


    public static double kP = 0.01, kI = 0.01, kD = 0.001, kF = 0.45, rotateSpeedDegPerSec = 220, drivePowerScale=1;

    private Movement movement;



    private double gatePose, hoodPose, rampPose;
    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();


        commands = new Commands(hardwareMap, telemetry);
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
                schedule(commands.startFlyWheel);
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

        if (gamepad1.dpadUpWasPressed()){
            commands.targetRpm = 2900;
        }
        if (gamepad1.dpadDownWasPressed()){
            commands.targetRpm = 1500;
        }
        if (gamepad1.dpadLeftWasPressed()){
            commands.targetRpm= 2200;
        }
        commands.updateFlywheel();   // в каждой итерации цикла

        telemetry.addData("target rpm", commands.getFlywheelTargetRpm());
        telemetry.addData("rpm", commands.getFlywheelRpm());
        telemetry.addData("atSpeed", commands.flywheelAtSpeed());

        Scheduler.execute();

        telemetry.addData("Hood:", hoodPose);
        telemetry.addData("Ramp:", rampPose);
        telemetry.addData("Gate:", gatePose);
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading", follower.pose().heading());
        telemetry.addData("fly wheel", commands.flyWheelRatio);
        telemetry.update();

    }
}