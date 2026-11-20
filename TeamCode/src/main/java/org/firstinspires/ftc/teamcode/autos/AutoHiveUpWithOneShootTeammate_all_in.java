package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.Storage;
import org.firstinspires.ftc.teamcode.pedro.Constants;


@Autonomous
public class AutoHiveUpWithOneShootTeammate_all_in extends OpMode {
    private Follower follower;
    private Commands commands;
    private ElapsedTime runtime;
    private double shootTime;
    public static double drivekP = 0.01, drivekI = 0.01, shootkP = 0.03, shootkI = 0.013, kD = 0.001, kF = 0, rotateSpeedDegPerSec = 220, drivePowerScale=1;
    private Movement movement;
    private double targetHeading;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    // Poses
    private final Pose startPose = poseFactory.of(8.21, 88.72, 180);
    private final Pose prepareIntakeGardenPose = poseFactory.of(20, 134.72, 180);
    private final Pose intakeGardenPose = poseFactory.of(10, 134.72, 180);
    private final Pose shootGardenPose = poseFactory.of(14, 104, 180);
    private final Pose prepareIntakeNearFlower = poseFactory.of(48, 120, 90);
    private final Pose intakeNearFlower = poseFactory.of(48, 130, 90);
    private final Pose prepareParkPose = poseFactory.of(48, 120, 90);
    private final Pose parkPose = poseFactory.of(100, 132, 0);
    private final Pose shootNearFlowerPose = poseFactory.of(120, 105, 45);
    private final Pose intakeFarFlowerPose = poseFactory.of(127, 96, 0);
    private final Pose shootFarFlowerPose = poseFactory.of(124, 88, 0);
    private final Pose prepareIntakeFarFlowerPose = poseFactory.of(117, 96, 0);
    // Path methods
    private Path prepareIntakeGarden() {
        return line(startPose, prepareIntakeGardenPose).linear(startPose, prepareIntakeGardenPose);
    }
    private Path shootGarden() {
        return line(intakeGardenPose, shootNearFlowerPose).linear(intakeGardenPose, shootNearFlowerPose);
    }
    private Path prepareIntakeNearFlower() {
        return line(shootGardenPose, prepareIntakeNearFlower).linear(shootGardenPose, prepareIntakeNearFlower);
    }
    private Path prepareShootNearFlower() {
        return line(intakeNearFlower, prepareParkPose).linear(intakeNearFlower, prepareParkPose);
    }
    private Path shootNearFlower() {
        return line(prepareParkPose, shootNearFlowerPose).linear(prepareParkPose, shootNearFlowerPose);
    }
    private Path prepareIntakeFarFlower() {
        return line(shootNearFlowerPose, prepareIntakeFarFlowerPose).linear(shootNearFlowerPose, prepareIntakeFarFlowerPose);
    }
    private Path shootFarFlower() {
        return line(intakeFarFlowerPose, shootGardenPose).linear(intakeFarFlowerPose, shootGardenPose);
    }
    private Path park() {
        return line(shootFarFlowerPose, parkPose).linear(shootFarFlowerPose, parkPose);
    }
    private Command autoRoutine;
    private Command driveOnTimer(double power, double timeMs) {
        ElapsedTime timer = new ElapsedTime();
        return Command.build()
                .setStart(() -> {
                    timer.reset();
                    follower.stop();
                }).setExecute(() -> {
                    waitMs(200);
                    movement.setPowerAll(power);
                })
                .setDone(() -> timer.milliseconds() >= timeMs)
                .setEnd(endCondition -> {
                    movement.setPowerAll(0);
                    follower.hold(follower.pose());
                });
    }
    private void buildCommands() {
        Command alignToShoot = Command.build()
                .setStart(() -> {
                    shootTime = runtime.milliseconds();
                    follower.stop();
                })
                .setExecute(() -> {
                    double currentHeadingDeg = movement.normalizeAngle(Math.toDegrees(follower.pose().heading()));
                    movement.align(currentHeadingDeg, targetHeading);
                })
                .setDone(() -> (runtime.milliseconds() - shootTime) >= 500)
                .setEnd(endCondition -> follower.hold(follower.pose()));

        Command shoot = sequential(
                commands.startIntake,
                commands.startTransfer,
                alignToShoot,
                commands.openGate,
                waitMs(1000)
        );

        Command stopShoot = sequential(
                commands.stopIntake,
                commands.stopTransfer,
                commands.closeGate
        );

        autoRoutine = sequential(
                commands.startFlyWheel,
                waitUntil(() -> commands.flywheelAtSpeed()),
                shoot,
                stopShoot,
                commands.aimLeftHive,
                follow(follower, prepareIntakeGarden()),
                commands.startIntake,
                commands.startTransfer,
                driveOnTimer(commands.FLOWER_INTAKE_POWER,2000),
                commands.stopIntake,
                commands.stopTransfer,
                follow(follower, shootGarden()),
                waitUntil(() -> commands.flywheelAtSpeed()),
                shoot,
                commands.lowerRamp,
                stopShoot,
                follow(follower, prepareIntakeNearFlower()),
                commands.startIntake,
                commands.startTransfer,
                driveOnTimer(commands.FLOWER_INTAKE_POWER,1000),
                waitMs(2000),
                commands.stopIntake,
                commands.stopTransfer,
                follow(follower, prepareShootNearFlower()),
                commands.raiseRamp,
                follow(follower, shootNearFlower()),
                waitUntil(() -> commands.flywheelAtSpeed()),
                shoot,
                stopShoot,
                commands.aimRightHive,
                commands.lowerRamp,
                commands.startIntake,
                commands.startTransfer,
                follow(follower, prepareIntakeFarFlower()),
                driveOnTimer(commands.FLOWER_INTAKE_POWER,1000),
                waitMs(2000),
                commands.startIntake,
                commands.startTransfer,
                follow(follower, shootFarFlower()),
                commands.raiseRamp,
                waitUntil(() -> commands.flywheelAtSpeed()),
                shoot,
                stopShoot,
                commands.stopFlyWheel
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
        commands = new Commands(hardwareMap, telemetry);
        runtime = new ElapsedTime();
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

        buildCommands();
        schedule(
                sequential(
                        commands.aimRightHive,
                        commands.closeGate,
                        commands.raiseRamp
                )
        );
        Scheduler.execute();
    }

    @Override
    public void start() {
        schedule(autoRoutine);
    }

    @Override
    public void loop() {
        follower.update();
        commands.updateFlywheel();
        targetHeading = commands.updateCalculations(follower.pose(), 0);
        Scheduler.execute();
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading", commands.normalizeAngle(Math.toDegrees(follower.pose().heading())));
        telemetry.addData("error", commands.normalizeAngle(Math.toDegrees(follower.pose().heading()) - targetHeading));
        telemetry.update();
    }

    @Override
    public void stop() {
        Storage.autonomousEndPose = follower.pose(); //saves your position in that file
    }
}