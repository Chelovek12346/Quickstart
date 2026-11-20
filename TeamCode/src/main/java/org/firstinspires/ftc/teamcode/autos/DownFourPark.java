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
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;


@Autonomous
public class DownFourPark extends OpMode {
    private Follower follower;
    private Commands commands;
    private ElapsedTime runtime;
    private double shootTime;
    public static double drivekP = 0.01, drivekI = 0.01, shootkP = 0.03, shootkI = 0.013, kD = 0.001, kF = 0, rotateSpeedDegPerSec = 220, drivePowerScale=1;
    private Movement movement;
    private double targetHeading;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    // Poses
    private final Pose startPose = poseFactory.of(144 - 8.21, 79.28, 0);
    private final Pose shootPreloadPose = poseFactory.of(144 - 8.21 - 10, 85, 0);
    private final Pose prepareIntakeFarFlowerPose = poseFactory.of(117, 96, 0);
    private final Pose intakeFarFlowerPose = poseFactory.of(127, 96, 0);
    private final Pose shootFarFlowerPose = poseFactory.of(117, 96, 0);
    private final Pose parkPose = poseFactory.of(96, 134.72, 0);

    // Path methods
    private Path shootPreload() {
        return line(startPose, shootPreloadPose).linear(startPose, shootPreloadPose);
    }
    private Path park() {
        return line(shootPreloadPose, parkPose).linear(shootPreloadPose, parkPose);
    }
    private Command autoRoutine;
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
                waitMs(15000),
                shoot,
                stopShoot,
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