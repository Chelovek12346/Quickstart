package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;


@Autonomous
public class TestAuto extends OpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    // Poses
    private final Pose startPose = poseFactory.of(8, 8, 0);
    private final Pose shootingPose1 = poseFactory.of(20, 60, 0);
    private final Pose shootingPose2 = poseFactory.of(132, 60, 180);
    private final Pose flowerPos1 = poseFactory.of(96, 30, 270);
    private final Pose flowerPodsos1 = poseFactory.of(96, 20, 270);
    private final Pose turningCorner1 = poseFactory.of(20, 20, 0);
    private final Pose turningCorner2 = poseFactory.of(126, 20, 90);
//    private final Pose startPose = poseFactory.of(6, 8, 0);
//    private final Pose shootingPose1 = poseFactory.of(12, 60, 0);
//    private final Pose shootingPose2 = poseFactory.of(122, 60, 0);
//    private final Pose flowerPos1 = poseFactory.of(96, 20, 0);
//    private final Pose flowerPodsos1 = poseFactory.of(96, 5, 0);
//    private final Pose turningCorner1 = poseFactory.of(20, 20, 0);
//    private final Pose turningCorner2 = poseFactory.of(124, 20, 0);
    // Path methods
    private Path shoot1() {
        return line(turningCorner1, shootingPose1).linear(turningCorner1, shootingPose1);
    }

    private Path shoot2() {
        return line(turningCorner2, shootingPose2).linear(turningCorner2, shootingPose2);
    }
    private Path ready(){
        return line(startPose, turningCorner1).linear(startPose, turningCorner1);
    }

    private Path shootCorner1(){
        return line(shootingPose1, turningCorner1).linear(shootingPose1, turningCorner1);
    }

    private Path toFlower1() {
        return line(turningCorner1, flowerPos1).linear(turningCorner1, flowerPos1);
    }

    private Path flowerPodsos1() {
        return line(flowerPos1, flowerPodsos1).linear(flowerPos1, flowerPodsos1);
    }

    private Path podsosComeback1() {
        return line(flowerPodsos1, flowerPos1).linear(flowerPodsos1, flowerPos1);
    }

    private Path flowerCorner2(){
        return line(flowerPos1, turningCorner2).linear(flowerPos1, turningCorner2);
    }



    private Command autoRoutine() {
        return sequential(
                follow(follower, ready()),
                follow(follower, shoot1()),
                waitMs(2000),
                follow(follower, shootCorner1()),
                follow(follower, toFlower1()),
                follow(follower,flowerPodsos1()),
                waitMs(2000),
                follow(follower,podsosComeback1()),
                follow(follower, flowerCorner2()),
                follow(follower, shoot2())


        );
    }
    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }
    @Override
    public void start() {
        schedule(autoRoutine());
    }
    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        // add your other methods needed in the loop here
    }
}