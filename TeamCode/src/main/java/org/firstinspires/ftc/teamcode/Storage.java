package org.firstinspires.ftc.teamcode;
//Make sure this matches your file location
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
public class Storage {
    private static final PoseFactory p = PoseFactory.degrees();
    public static Pose autonomousEndPose = p.of(8.21, 88.72, 180);
    // default pose to avoid null errors
}