package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.pedropathing.algorithm.ForesightConfig;

import com.pedropathing.controllers.Controller;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("leftFront");
                c.backLeftName.set("leftRear");
                c.frontRightName.set("rightFront");
                c.backRightName.set("rightRear");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

                c.manualBrakeMode.set(true);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");

                c.xPodOffset.set(5.2402);
                c.yPodOffset.set(-1.7205);

                //forwardPodY = 133.1 mm
                //strafePodX = -43.7 mm
                //Pedro 3 usa los offsets en pulgadas, así que:
                //133.1 mm ÷ 25.4 = 5.2402 in
                //−43.7 mm ÷ 25.4 = −1.7205 in

                c.xPodDirection.set(
                        GoBildaPinpointDriver.EncoderDirection.REVERSED
                );

                c.yPodDirection.set(
                        GoBildaPinpointDriver.EncoderDirection.FORWARD
                );
            }
    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                // Velocidades máximas
                c.maxAchievableForwardVelocity.set(79.1633);
                c.maxAchievableStrafeVelocity.set(54.95642173947312);

                // Desaceleración natural
                c.naturalForwardDeceleration.set(43.4139961087975);
                c.naturalStrafeDeceleration.set(54.89034947229055);

                // Translational PID
                c.forwardTranslational.set(
                        Controller.pid(0.02, 0, 0.002)
                );

                c.strafeTranslational.set(
                        Controller.pid(0.02, 0, 0.002)
                );

                // Heading PID
                c.headingFeedback.set(
                        Controller.pid(0.83, 0, 0.004)
                );

                // Restricciones de finalización
                c.parametricTConstraint.set(0.025);
                c.velocityConstraint.set(0.1);
                c.translationalConstraint.set(0.1);
                c.headingConstraint.set(0.007);
                c.timeoutConstraint.set(100.0);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}