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
import org.firstinspires.ftc.teamcode.pedro.Constants;


@Autonomous
public class TestAuto extends OpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    // Poses
    private final Pose startPose = poseFactory.of(144-7.1, 57, 180);
    private final Pose shootingPose1 = poseFactory.of(144-7.1, 57, 180);
    private final Pose shootingPose2 = poseFactory.of(20, 57, 0);
    private final Pose flowerPos1 = poseFactory.of(96, 15, 270);
    private final Pose flowerIntake1 = poseFactory.of(96, 9, 270);
    private final Pose turningCorner1 = poseFactory.of(120, 24, 180);
    private final Pose turningCorner2 = poseFactory.of(24, 24, 90);
    private final Pose flowerPos2 = poseFactory.of(15, 48, 180);
    private final Pose flowerIntake2 = poseFactory.of(9, 48, 180);
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

    private Path shootCorner2(){
        return line(shootingPose2, turningCorner2).linear(shootingPose2, turningCorner2);
    }

    private Path toFlower1() {
        return line(turningCorner1, flowerPos1).linear(turningCorner1, flowerPos1);
    }

    private Path flowerIntake1() {
        return line(flowerPos1, flowerIntake1).linear(flowerPos1, flowerIntake1).with(Constants.foresightConfig.maxPathSpeed.at(0.5));
    }

    private Path podsosComeback1() {
        return line(flowerIntake1, flowerPos1).linear(flowerIntake1, flowerPos1);
    }

    private Path flower1Corner2(){
        return line(flowerPos1, turningCorner2).linear(flowerPos1, turningCorner2);
    }

    private Path flower2Corner2(){
        return line(flowerPos2, turningCorner2).linear(flowerPos2, turningCorner2);
    }

    private Path toFlower2() {
        return line(turningCorner2, flowerPos2).linear(turningCorner2, flowerPos2);
    }

    private Path flowerIntake2() {
        return line(flowerPos2, flowerIntake2).linear(flowerPos2, flowerIntake2).with(Constants.foresightConfig.maxPathSpeed.at(0.5));
    }

    private Path podsosComeback2() {
        return line(flowerIntake2, flowerPos2).linear(flowerIntake2, flowerPos2);
    }



    private Command autoRoutine() {
        return sequential(
                // shoot
                waitMs(2000),
                follow(follower, shootCorner1()),
                follow(follower, toFlower1()),
                // lower ramp
//                instant(() -> follower.)
                follow(follower,flowerIntake1()),
                waitMs(2000),
                follow(follower,podsosComeback1()),
                // raise ramp
                follow(follower, flower1Corner2()),
                follow(follower, shoot2()),
                // shoot
                waitMs(2000),
                follow(follower, shootCorner2()),
                follow(follower, toFlower2()),
                // lower ramp
                follow(follower,flowerIntake2()),
                waitMs(2000),
                follow(follower,podsosComeback2()),
                // raise ramp
                follow(follower, flower2Corner2()),
                follow(follower, shoot2()),
                // shoot
                waitMs(2000),
                follow(follower, shootCorner2())
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