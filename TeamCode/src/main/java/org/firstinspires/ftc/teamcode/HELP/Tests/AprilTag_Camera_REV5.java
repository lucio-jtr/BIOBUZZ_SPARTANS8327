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

import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import java.util.List;
import java.util.Locale;

@Disabled
@TeleOp(name = "AprilTag_Camera_REV5")
public class AprilTag_Camera_REV5 extends OpMode {

    private Servo servo_pos;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final boolean USE_WEBCAM = true;

    // Posición inicial
    double position = 0.50;

    // Sensibilidad experimental
    double sensitivity = 490.0;

    // Offset del centro físico de la CELL
    double centerOffset = 8.0;

    // Límites del servo
    double minPosition = 0.44;
    double maxPosition = 0.63;

    // Bearing
    double bearing = 0;
    double correctedBearing = 0;
    double filteredBearing = 0;

    // Controlador
    boolean correcting = false;
    boolean firstDetection = true;

    // Actualización
    double updateInterval = 0.04;
    double lastUpdate = 0;

    @Override
    public void init() {

        // Biblioteca oficial BIOBUZZ
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(
                        AprilTagGameDatabase.getBioBuzzTagLibrary()
                )
                .setLensIntrinsics(
                        1432.032,
                        1432.032,
                        997.085,
                        535.891
                )
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal.Builder builder =
                new VisionPortal.Builder()
                        .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                        .setCameraResolution(new Size(1280, 800));

        if (USE_WEBCAM) {

            builder.setCamera(
                    hardwareMap.get(WebcamName.class, "Webcam 1")
            );

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

        AprilTagClusterDetection target = null;

        // Buscar Blue Scoring
        for (AprilTagDetection detection : detections) {

            if (detection instanceof AprilTagClusterDetection
                    && detection.ftcPose != null) {

                AprilTagClusterDetection cluster =
                        (AprilTagClusterDetection) detection;

                String name =
                        cluster.metadata.name.toUpperCase(Locale.ROOT);

                if (name.contains("BLUE SCORING")) {

                    target = cluster;
                    break;
                }
            }
        }

        if (target != null) {

            // Bearing original
            bearing = target.ftcPose.bearing;

            // Aplicar offset del centro físico
            correctedBearing = bearing - centerOffset;

            // Filtrado
            if (firstDetection) {

                filteredBearing = correctedBearing;
                firstDetection = false;

            } else {

                filteredBearing =
                        0.7 * filteredBearing
                                + 0.3 * correctedBearing;
            }

            // Actualización controlada
            if (getRuntime() - lastUpdate >= updateInterval) {

                lastUpdate = getRuntime();

                double absError =
                        Math.abs(filteredBearing);

                // Histéresis
                if (!correcting && absError > 1.5) {
                    correcting = true;
                }

                if (correcting && absError < 0.7) {
                    correcting = false;
                }

                if (correcting) {

                    double adaptiveKp;
                    double maxStepNow;

                    if (absError > 15) {

                        adaptiveKp = 0.45;
                        maxStepNow = 0.0008;

                    } else if (absError > 7) {

                        adaptiveKp = 0.30;
                        maxStepNow = 0.00045;

                    } else if (absError > 3) {

                        adaptiveKp = 0.15;
                        maxStepNow = 0.00020;

                    } else {

                        adaptiveKp = 0.06;
                        maxStepNow = 0.00008;
                    }

                    // Corrección proporcional
                    double correction =
                            (filteredBearing / sensitivity)
                                    * adaptiveKp;

                    // Limitar movimiento
                    correction = Math.max(
                            -maxStepNow,
                            Math.min(maxStepNow, correction)
                    );

                    // Aplicar corrección
                    position -= correction;

                    // Límites mecánicos
                    position = Math.max(
                            minPosition,
                            Math.min(maxPosition, position)
                    );

                    servo_pos.setPosition(position);
                }
            }

            // TELEMETRY
            telemetry.addLine("=== BLUE SCORING ===");

            telemetry.addData(
                    "Cluster",
                    target.metadata.name
            );

            telemetry.addData(
                    "Tags visibles",
                    target.percentClusterFound
            );

            telemetry.addData(
                    "Bearing original",
                    "%.2f", bearing
            );

            telemetry.addData(
                    "Offset",
                    "%.2f", centerOffset
            );

            telemetry.addData(
                    "Bearing corregido",
                    "%.2f", correctedBearing
            );

            telemetry.addData(
                    "Bearing filtrado",
                    "%.2f", filteredBearing
            );

            telemetry.addData(
                    "Error absoluto",
                    "%.2f", Math.abs(filteredBearing)
            );

            telemetry.addData(
                    "Corrigiendo",
                    correcting
            );

            telemetry.addData(
                    "Position servo",
                    "%.5f", position
            );

        } else {

            // Conservar posición si se pierde el clúster
            firstDetection = true;
            correcting = false;

            telemetry.addLine("BLUE SCORING NO DETECTADO");

            telemetry.addData(
                    "Position conservada",
                    "%.5f", position
            );
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