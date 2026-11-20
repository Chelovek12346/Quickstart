package org.firstinspires.ftc.teamcode.teleOps;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.Storage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Teleop")
public class Teleop extends OpMode {
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private Follower follower;
    private Commands commands;
    private boolean flyWheelState, hoodState, rampState;
    public static double drivekP = 0.01, drivekI = 0.007, shootkP = 0.03, shootkI = 0.013, kD = 0.001, kF = 0.45, rotateSpeedDegPerSec = 220, drivePowerScale=1;

    private Movement movement;

    private double targetHeading = 0, headingTrimDeg, distanceTrimInches;

    private boolean shooting, flyWheelToggle;

    private ElapsedTime runtime;
    private double shootingTime = 0;

    private double gatePose, hoodPose, rampPose;
    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(Storage.autonomousEndPose);
        follower.update();

        movement = new Movement(
                hardwareMap,
                drivekP,
                drivekI,
                shootkP,
                shootkI,
                kD,
                kF,
                rotateSpeedDegPerSec,
                drivePowerScale,
                Math.toDegrees(follower.pose().heading())
        );

        runtime = new ElapsedTime();

        commands = new Commands(hardwareMap, telemetry);
        flyWheelToggle = true;
        hoodState = false;
        rampState = false;
        shooting = false;

        schedule(commands.closeGate);
        schedule(commands.raiseHood);
        schedule(commands.raiseRamp);
        schedule(commands.startFlyWheel);

        rampPose = 0.5;
        headingTrimDeg = 0;
        distanceTrimInches = 0;

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
        double cos = Math.cos(-(heading + Math.PI / 2));
        double sin = Math.sin(-(heading + Math.PI / 2));
        double xField = strafe * cos - forward * sin;
        double yField = strafe * sin + forward * cos;

        commands.updateFlywheel();
        targetHeading = commands.updateCalculations(follower.pose(), distanceTrimInches);
        targetHeading += headingTrimDeg;

        movement.drive(
                xField,
                yField,
                gamepad1.right_stick_x,    // стик вправо
                -gamepad1.right_stick_y,   // стик вверх
                Math.toDegrees(heading),
                shooting,
                targetHeading
        );

        follower.update();

        boolean isLeftHive = follower.pose().x() > 72;
        if (isLeftHive) {
            schedule(commands.aimLeftHive);
            gamepad1.setLedColor(1.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(1.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            schedule(commands.aimRightHive);
            gamepad1.setLedColor(0.0, 0.0, 1.0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(0.0, 0.0, 1.0, Gamepad.LED_DURATION_CONTINUOUS);
        }

        if (gamepad1.rightTriggerWasPressed()) {
            shooting = true;
            schedule(commands.startIntake);
            schedule(commands.startTransfer);
        }

        if (shooting && Math.abs(movement.normalizeAngle(movement.normalizeAngle(Math.toDegrees(heading)) - targetHeading)) > 10) {
            shootingTime = runtime.milliseconds();
        }

        if (shooting && runtime.milliseconds() - shootingTime > 300) {
            schedule(commands.openGate);
        }

        if (gamepad1.rightTriggerWasReleased()) {
            shooting = false;
            schedule(commands.closeGate);
            schedule(commands.stopIntake);
            schedule(commands.stopTransfer);
        }

        if (gamepad1.rightStickButtonWasPressed()) {
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

        if (gamepad1.leftTriggerWasReleased() || gamepad1.leftBumperWasReleased()) {
            schedule(commands.stopIntake);
            schedule(commands.stopTransfer);
        }

        if (gamepad1.leftBumperWasPressed()) {
            schedule(commands.reverseIntake);
            schedule(commands.reverseTransfer);
        }

        if (gamepad2.dpadLeftWasPressed()) {
            headingTrimDeg += 1;
        }
        if (gamepad2.dpadRightWasPressed()) {
            headingTrimDeg -= 1;
        }

        if (gamepad2.dpadUpWasPressed()) {
            distanceTrimInches += 0.5;
        }
        if (gamepad2.dpadDownWasPressed()) {
            distanceTrimInches -= 0.5;
        }

        if (gamepad1.dpadLeftWasPressed() && Math.hypot(follower.pose().x() - 8, follower.pose().y() - 136.72) < 5 && Math.abs(movement.normalizeAngle(Math.toDegrees(heading))) < 4 ){
            follower.setPose(poseFactory.of(8, 136.72, 0));
        }

//        if (gamepad1.dpadUpWasPressed()) {
//            commands.targetRpm += 50;
//        }
//
//        if (gamepad1.dpadDownWasPressed()) {
//            commands.targetRpm -= 50;
//        }
//
//        if (gamepad1.dpadLeftWasPressed()) {
//            commands.targetRpm -= 10;
//        }
//
//        if (gamepad1.dpadRightWasPressed()) {
//            commands.targetRpm += 10;
//        }

        Scheduler.execute();

        telemetry.addData("Target Heading:", targetHeading);
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading", commands.normalizeAngle(Math.toDegrees(heading)));
        telemetry.update();

    }
}