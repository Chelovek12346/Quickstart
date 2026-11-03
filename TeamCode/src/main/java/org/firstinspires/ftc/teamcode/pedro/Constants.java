package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeftMotor");
        c.frontRightName.set("frontRightMotor");
        c.backLeftName.set("backLeftMotor");
        c.backRightName.set("backRightMotor");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(4.5);
        c.yPodOffset.set(2.7);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3689814308482122);
                Controller secondaryTranslationalForward = Controller.proportional(0.13632877933299964);
                Controller primaryTranslationalLateral = Controller.proportional(0.49635362795760823);
                Controller secondaryTranslationalLateral = Controller.proportional(0.18338940271713253);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013681318958183612));
                c.brake.set(Controller.proportionalFeedforward(0.01162912111445607));

                c.headingFeedback.set(Controller.proportional(5.370523258139332));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.11818883270540487, 0.0015802511430647785));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06950622309157124, 0.029920832438913234));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.00220327517928927, 0.0024046838072759504));

                c.maxAchievableForwardVelocity.set(69.07861200093359);
                c.maxAchievableStrafeVelocity.set(58.87835263263371);
                c.naturalForwardDeceleration.set(37.544072352829396);
                c.naturalStrafeDeceleration.set(66.14778308498583);
            }
    );

//    public static ForesightConfig foresightConfig = new ForesightConfig(
//            c -> {
//                Controller primaryTranslationalForward = Controller.proportional(0.34975430536270324);
//                Controller secondaryTranslationalForward = Controller.proportional(0.12922487022435905);
//                Controller primaryTranslationalLateral = Controller.proportional(0.497495099863073);
//                Controller secondaryTranslationalLateral = Controller.proportional(0.18381114608551063);
//
//                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
//                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
//
//                c.coast.set(Controller.proportionalFeedforward(0.013613851753624845));
//                c.brake.set(Controller.proportionalFeedforward(0.011571773990581118));
//
//                c.headingFeedback.set(Controller.proportional(6.36303179907111));
//                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06234303470970999, 0.0063657851100393315));
//
//                c.linearBrakeCoefficients.set(Matrix.diag(0.11649286263289317, 0.05443912768259257));
//                c.quadraticBrakeCoefficients.set(Matrix.diag(6.399296088200501E-4, 0.0019522717977095024));
//
//                c.maxAchievableForwardVelocity.set(72.14180309704588);
//                c.maxAchievableStrafeVelocity.set(61.678259644774776);
//                c.naturalForwardDeceleration.set(37.00321203832141);
//                c.naturalStrafeDeceleration.set(66.70112753363382);
//            }
//    );

//    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
//        c.frontLeftName.set("frontLeftMotor");
//        c.frontRightName.set("frontRightMotor");
//        c.backLeftName.set("backLeftMotor");
//        c.backRightName.set("backRightMotor");
//        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
//        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
//        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
//        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
//    });

//    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
//        c.frontLeftName.set("frontLeftMotor");
//        c.frontRightName.set("frontRightMotor");
//        c.backLeftName.set("backLeftMotor");
//        c.backRightName.set("backRightMotor");
//        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
//        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
//        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
//        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
//    });
//    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
//        c.name.set("pinpoint");
//        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        c.xPodOffset.set(3.5201056172528604);
//        c.yPodOffset.set(-3.108276006743664);
//        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        c.globalDistanceUnit.set(DistanceUnit.INCH);
//        c.offsetUnits.set(DistanceUnit.INCH);
//    });

//    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
//        c.name.set("pinpoint");
//        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        c.xPodOffset.set(4.25);
//        c.yPodOffset.set(3.8);
//        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        c.globalDistanceUnit.set(DistanceUnit.INCH);
//        c.offsetUnits.set(DistanceUnit.INCH);
//    });

//    public static ForesightConfig foresightConfig = new ForesightConfig(
//            c -> {
//                Controller primaryTranslationalForward = Controller.proportional(0.2160481207485867);
//                Controller secondaryTranslationalForward = Controller.proportional(0.07982400770449516);
//                Controller primaryTranslationalLateral = Controller.proportional(0.2609109247446694);
//                Controller secondaryTranslationalLateral = Controller.proportional(0.09639961502484713);
//
//                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
//                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
//
//                c.coast.set(Controller.proportionalFeedforward(0.014743311580365393));
//                c.brake.set(Controller.proportionalFeedforward(0.012531814843310583));
//
//                c.headingFeedback.set(Controller.proportional(3.404539023928076));
//                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.03690427614375376, 0.009122479554604686));
//
//                c.linearBrakeCoefficients.set(Matrix.diag(0.044752144225085534, 0.06523695799093998));
//                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0020912359011198423, 0.0017357365339391764));
//
//                c.maxAchievableForwardVelocity.set(74.27147232181487);
//                c.maxAchievableStrafeVelocity.set(62.848817195880066);
//                c.naturalForwardDeceleration.set(77.20363015892893);
//                c.naturalStrafeDeceleration.set(90.89517256400343);
//            }
//    );
//    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
//        c.name.set("pinpoint");
//        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        c.xPodOffset.set(3.6601960189699185);
//        c.yPodOffset.set(-2.1580433282326528);
//        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        c.globalDistanceUnit.set(DistanceUnit.INCH);
//        c.offsetUnits.set(DistanceUnit.INCH);
//    });

//    public static ForesightConfig foresightConfig = new ForesightConfig(
//            c -> {
//                Controller primaryTranslationalForward = Controller.proportional(0.2185486451773154);
//                Controller secondaryTranslationalForward = Controller.proportional(0.0807478846656671);
//                Controller primaryTranslationalLateral = Controller.proportional(0.2571825610961461);
//                Controller secondaryTranslationalLateral = Controller.proportional(0.09502208428042924);
//
//                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
//                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
//
//                c.coast.set(Controller.proportionalFeedforward(0.014105528468074445));
//                c.brake.set(Controller.proportionalFeedforward(0.011989699197863277));
//
//                c.headingFeedback.set(Controller.proportional(2.8665971903951872));
//                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.047426480345518635, 0.006617121927697924));
//
//                c.linearBrakeCoefficients.set(Matrix.diag(0.04024885542903689, 0.04215138399495574));
//                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0023319247876463067, 0.0020498760062143875));
//
//                c.maxAchievableForwardVelocity.set(76.55707462279943);
//                c.maxAchievableStrafeVelocity.set(63.574590530451836);
//                c.naturalForwardDeceleration.set(83.25746332055411);
//                c.naturalStrafeDeceleration.set(100.18633502827723);
//            }
//    );
}