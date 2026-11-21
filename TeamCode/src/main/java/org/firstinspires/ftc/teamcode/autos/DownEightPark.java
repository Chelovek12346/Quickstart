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

//

//Ждем вначале чтобы альянс перевернул, мы с нидней стороны

@Autonomous
public class DownEightPark extends OpMode {
    private Follower follower;
    private Commands commands;
    private ElapsedTime runtime;
    private double shootTime;
    public static double drivekP = 0.01, drivekI = 0.01, shootkP = 0.03, shootkI = 0.013, kD = 0.001, kF = 0,
            rotateSpeedDegPerSec = 220, drivePowerScale=1;
    private Movement movement;
    private double targetHeading;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    // Poses
    private final Pose startPose = poseFactory.of(144 - 8.21, 79.28, 0);
    private final Pose shootPreloadPose = poseFactory.of(144 - 8.21 - 5, 79.28, 0);
    private final Pose prepareIntakeFarFlowerPose = poseFactory.of(117, 96, 0);
    private final Pose intakeFarFlowerPose = poseFactory.of(132, 96, 0);
    private final Pose shootFarFlowerPose = poseFactory.of(144 - 8.21 - 5, 79.28, 0);
    private final Pose parkPose = poseFactory.of(125, 132.72, 0);

    // Path methods
    private Path shootPreload() {
        return line(startPose, shootPreloadPose).linear(startPose, shootPreloadPose);
    }
    private Path prepareIntakeFarFlower() {
        return line(shootPreloadPose, prepareIntakeFarFlowerPose).linear(shootPreloadPose, prepareIntakeFarFlowerPose);
    }
    private Path intakeFarFlower() {
        return line(prepareIntakeFarFlowerPose, intakeFarFlowerPose).linear(prepareIntakeFarFlowerPose, intakeFarFlowerPose);
    }
    private Path shootFarFlower() {
        return line(prepareIntakeFarFlowerPose, shootFarFlowerPose).linear(prepareIntakeFarFlowerPose, shootFarFlowerPose);
    }

    private Path backFarFlower() {
        return line(intakeFarFlowerPose, prepareIntakeFarFlowerPose).linear(intakeFarFlowerPose, prepareIntakeFarFlowerPose);
    }

    private Path preparePark() {
        return line(shootFarFlowerPose, prepareIntakeFarFlowerPose).linear(shootFarFlowerPose, prepareIntakeFarFlowerPose);
    }
    private Path park() {
        return line(shootPreloadPose, parkPose).linear(shootPreloadPose, parkPose);
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
                    if (Math.abs(movement.normalizeAngle(targetHeading - currentHeadingDeg)) > 3) {
                        shootTime = runtime.milliseconds();
                    }
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
                follow(follower, shootPreload()),
                waitMs(10000),
                shoot,
                stopShoot,
//                commands.lowerRamp,
//                follow(follower, prepareIntakeFarFlower()),
//                commands.startIntake,
//                commands.startTransfer,
//                driveOnTimer(commands.FLOWER_INTAKE_POWER,1500),
//                waitMs(1500),
//                commands.stopIntake,
//                commands.stopTransfer,
//                follow(follower,backFarFlower()),
//                follow(follower, shootFarFlower()),
//
//                waitUntil(() -> commands.flywheelAtSpeed()),
//                stopShoot,commands.raiseRamp,
//                shoot,
//                follow(follower, preparePark()),
                follow(follower, park()),
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
                        commands.aimLeftHive,
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