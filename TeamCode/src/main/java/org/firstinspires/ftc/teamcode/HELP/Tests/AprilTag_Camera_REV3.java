package org.firstinspires.ftc.teamcode.HELP.Tests;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.util.List;

@Disabled
@TeleOp(name = "AprilTag_Camera_REV3")
public class AprilTag_Camera_REV3 extends OpMode {

    private Servo servo_pos;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final boolean USE_WEBCAM = true;

    // Posicion inicial
    double position = 0.50;

    // Sensibilidad experimental
    double sensitivity = 490.0;

    // Controlador
    double Kp = 0.18;
    double deadband = 0.7;

    // Limites
    double minPosition = 0.44;
    double maxPosition = 0.63;

    // Seguridad de movimiento
    double maxStep = 0.0003;
    double updateInterval = 0.08;

    double bearing = 0;
    double filteredBearing = 0;

    double lastUpdate = 0;

    boolean firstDetection = true;

    @Override
    public void init() {

        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(
                        1432.032, 1432.032,
                        997.085, 535.891)
                .build();

        VisionPortal.Builder builder =
                new VisionPortal.Builder()
                        .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                        .setCameraResolution(new Size(1280, 800));

        if (USE_WEBCAM) {
            builder.setCamera(
                    hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(aprilTag);

        visionPortal = builder.build();

        servo_pos = hardwareMap.get(Servo.class, "servo");

        position = 0.50;
        servo_pos.setPosition(position);
    }

    @Override
    public void loop() {

        List<AprilTagDetection> detections =
                aprilTag.getDetections();

        AprilTagDetection target = null;

        // Buscar clúster
        for (AprilTagDetection detection : detections) {

            if (detection instanceof AprilTagClusterDetection
                    && detection.ftcPose != null) {

                target = detection;
                break;
            }
        }

        // Buscar tag individual
        if (target == null) {

            for (AprilTagDetection detection : detections) {

                if (detection.ftcPose != null) {
                    target = detection;
                    break;
                }
            }
        }

        if (target != null) {

            bearing = target.ftcPose.bearing;

            // Filtrar pequeñas variaciones
            if (firstDetection) {
                filteredBearing = bearing;
                firstDetection = false;
            } else {
                filteredBearing =
                        0.7 * filteredBearing + 0.3 * bearing;
            }

            // Actualizar solamente cada 100 ms
            if (getRuntime() - lastUpdate >= updateInterval) {

                lastUpdate = getRuntime();

                if (Math.abs(filteredBearing) > deadband) {

                    // Sensibilidad y correccion proporcional
                    double correction =
                            (filteredBearing / sensitivity) * Kp;

                    // Limitar movimiento
                    correction = Math.max(
                            -maxStep,
                            Math.min(maxStep, correction)
                    );

                    // Posicion aumenta cuando bearing disminuye
                    position -= correction;

                    // Limites del servo
                    position = Math.max(
                            minPosition,
                            Math.min(maxPosition, position)
                    );

                    servo_pos.setPosition(position);
                }
            }

            telemetry.addData("ESTADO", "DETECTADO");
            telemetry.addData("Bearing original", "%.2f", bearing);
            telemetry.addData("Bearing filtrado", "%.2f",
                    filteredBearing);
            telemetry.addData("Position", "%.5f", position);
            telemetry.addData("Servo", "%.5f",
                    servo_pos.getPosition());

        } else {

            // No mover el servo cuando se pierde el objetivo
            firstDetection = true;

            telemetry.addData("ESTADO", "TAG PERDIDO");
            telemetry.addData("Position conservada", "%.5f",
                    position);
        }

        telemetry.update();
    }

    @Override
    public void stop() {

        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}