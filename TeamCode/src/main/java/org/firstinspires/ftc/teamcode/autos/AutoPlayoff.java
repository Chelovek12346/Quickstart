package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Commands;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.Storage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;


@Autonomous
public class AutoPlayoff extends OpMode {
    private Follower follower;
    private Commands commands;
    private ElapsedTime runtime;
    private double shootTime;
    public static double drivekP = 0.01, drivekI = 0.01, shootkP = 0.03, shootkI = 0.013, kD = 0.001, kF = 0, rotateSpeedDegPerSec = 220, drivePowerScale=1;
    private Movement movement;
    private double targetHeading;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    // Poses
    private final Pose startPose = poseFactory.of(8.21, 112.72, 180);
    private final Pose startshootPose = poseFactory.of(15, 112.72, 160);
    private final Pose turnPose = poseFactory.of(13, 84, 180);
    private final Pose prepareIntakeGardenPose = poseFactory.of(20, 134.72, 188);
    private final Pose intakeGardenPose = poseFactory.of(10, 134.72, 180);
    private final Pose shootGardenPose = poseFactory.of(122,76 , 0);
    private final Pose prepareIntakeNearFlower = poseFactory.of(44, 120, 90);
    private final Pose intakeNearFlower = poseFactory.of(46, 130, 90);
    private final Pose prepareParkPose = poseFactory.of(48, 120, 90);
    private final Pose parkPose = poseFactory.of(120, 132, 0);
    private final Pose shootNearFlowerPose = poseFactory.of(130, 80, 0);
    private final Pose preshootNearFlowerPose = poseFactory.of(110, 80, 0);
    private final Pose prepareShootNearFlowerPose = poseFactory.of(110, 115, 0);
    private final Pose intakeFarFlowerPose = poseFactory.of(127, 102, 0);
    private final Pose shootFarFlowerPose = poseFactory.of(124, 88, 0);
    private final Pose prepareIntakeFarFlowerPose = poseFactory.of(117, 94, 0);

    // Path methods
    private Path startShoot() {
        return line(startPose, startshootPose).linear(startPose, startshootPose);
    }
    private Path prepareIntakeGarden() {
        return line(startPose, prepareIntakeGardenPose).linear(startPose, prepareIntakeGardenPose);
    }
    private Path intakeGarden() {
        return line(prepareIntakeGardenPose, intakeGardenPose).linear(prepareIntakeGardenPose, intakeGardenPose);
    }
    private Path turnGarden() {
        return line(intakeGardenPose, turnPose).linear(intakeGardenPose, turnPose);
    }
    private Path shootGarden() {
        return line(startPose, shootGardenPose).linear(startPose, shootGardenPose);
    }
    private Path prepareIntakeNearFlower() {
        return line(startshootPose, prepareIntakeNearFlower).linear(startshootPose, prepareIntakeNearFlower);
    }

    private Path backIntakeNearFlower() {
        return line(intakeNearFlower, prepareIntakeNearFlower).linear(intakeNearFlower, prepareIntakeNearFlower);
    }
    private Path intakeNearFlower() {
        return line(prepareIntakeNearFlower, intakeNearFlower).linear(prepareIntakeNearFlower, intakeNearFlower);
    }
    private Path prepareShootNearFlower() {
        return line(prepareIntakeNearFlower, prepareShootNearFlowerPose).linear(prepareIntakeNearFlower, prepareShootNearFlowerPose);
    }
    private Path prepareShootNearFlower2() {
        return line(prepareShootNearFlowerPose, preshootNearFlowerPose).linear(prepareShootNearFlowerPose, preshootNearFlowerPose);
    }
    private Path shootNearFlower() {
        return line(preshootNearFlowerPose, shootNearFlowerPose).linear(preshootNearFlowerPose, shootNearFlowerPose);
    }
    private Path prepareIntakeFarFlower() {
        return line(preshootNearFlowerPose, prepareIntakeFarFlowerPose).linear(preshootNearFlowerPose, prepareIntakeFarFlowerPose);
    }
    private Path backIntakeFarFlower() {
        return line(intakeFarFlowerPose, prepareIntakeFarFlowerPose).linear(intakeFarFlowerPose, prepareIntakeFarFlowerPose);
    }
    private Path intakeFarFlower() {
        return line(prepareIntakeFarFlowerPose, intakeFarFlowerPose).linear(prepareIntakeFarFlowerPose, intakeFarFlowerPose);
    }
    private Path shootFarFlower() {
        return line(prepareIntakeFarFlowerPose, preshootNearFlowerPose).linear(prepareIntakeFarFlowerPose, preshootNearFlowerPose);
    }
    private Path preparePark() {
        return line(shootNearFlowerPose, preshootNearFlowerPose).linear(shootNearFlowerPose, preshootNearFlowerPose);
    }
    private Path park() {
        return line(preshootNearFlowerPose, parkPose).linear(preshootNearFlowerPose, parkPose);
    }
    private Command autoRoutine;
    private Command driveOnTimer(double power, double timeMs, double targetHeadingDeg) {
        ElapsedTime timer = new ElapsedTime();
        return Command.build()
                .setStart(() -> {
                    timer.reset();
                    follower.stop();

                })
                .setExecute(() -> {
                    double headingDeg = Math.toDegrees(follower.pose().heading());
                    double error = movement.normalizeAngle(headingDeg - targetHeadingDeg);
                    double turn = commands.HEADING_KP * error;

                    double powerL = power + turn;
                    double powerR = power - turn;

                    // чтобы не вылезти за 1 и сохранить соотношение сторон
                    double max = Math.max(1, Math.max(Math.abs(powerL), Math.abs(powerR)));
                    movement.setPowerLeftRight(powerL / max, powerR / max);
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
                waitMs(1200)
        );

        Command shootNoAlign = sequential(
                commands.startIntake,
                commands.startTransfer,
                commands.openGate,
                waitMs(1200)
        );

        Command stopShoot = sequential(
                commands.stopIntake,
                commands.stopTransfer,
                commands.closeGate
        );

        autoRoutine = sequential(
                commands.startFlyWheel,
                follow(follower, startShoot()),
                waitUntil(() -> commands.flywheelAtSpeed()),
                shoot,
                commands.lowerRamp,
                stopShoot,
                commands.reverseIntake,
                commands.reverseTransfer,
                follow(follower, prepareIntakeNearFlower()),
                commands.startIntake,
                commands.startTransfer,
                waitMs(200),
                driveOnTimer(commands.FLOWER_INTAKE_POWER,1000,90),
                waitMs(2000),
                follow(follower,backIntakeNearFlower()),

                commands.aimLeftHive,
                follow(follower,prepareShootNearFlower()),
                commands.stopIntake,
                commands.stopTransfer,
                follow(follower,prepareShootNearFlower2()),
                commands.raiseRamp,
                follow(follower, shootNearFlower()),
                waitUntil(() -> commands.flywheelAtSpeed()),
                shoot,
                stopShoot,
                commands.reverseIntake,
                commands.reverseTransfer,

//                follow(follower, prepareIntakeNearFlower()),
//                commands.startIntake,
//                commands.startTransfer,
//                waitMs(200),
//                driveOnTimer(commands.FLOWER_INTAKE_POWER,1000),
//                waitMs(2000),
//                commands.stopIntake,
//                commands.stopTransfer,
//                follow(follower, prepareIntakeFarFlower()),
//                commands.raiseRamp,
//                follow(follower, shootNearFlower()),
//                waitUntil(() -> commands.flywheelAtSpeed()),
//                shoot,
//                stopShoot,
                follow(follower, preparePark()),
                commands.lowerRamp,
                follow(follower, prepareIntakeFarFlower()),
                commands.startIntake,
                commands.startTransfer,
                waitMs(200),
                driveOnTimer(commands.FLOWER_INTAKE_POWER,1000,0),
                waitMs(2000),
                follow(follower,backIntakeFarFlower()),
                commands.stopIntake,
                commands.stopTransfer,
                commands.raiseRamp,
                follow(follower, shootFarFlower()),
                follow(follower,shootNearFlower()),

                waitUntil(() -> commands.flywheelAtSpeed()),
                shoot,
                stopShoot,
                commands.stopFlyWheel,
                follow(follower, preparePark()),
                follow(follower, park())
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