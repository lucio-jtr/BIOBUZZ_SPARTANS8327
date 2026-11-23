package org.firstinspires.ftc.teamcode.HELP.Tests;

import android.util.Size;

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

@TeleOp(name = "AprilTag_Camera_Chasis_REV1")
public class AprilTag_Camera_Chasis_REV1 extends OpMode {

    private Servo turret;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final boolean USE_WEBCAM = true;

    // Posición inicial
    double position = 0.50;

    // Sensibilidad experimental
    double sensitivity = 490.0;

    // Offset del centro físico de la CELL
    double centerOffset = 8.0;

    // Relación de engranajes
    // El turret gira 2 veces por cada vuelta de cámara
    double gearRatio = 2.0;

    // Límites mecánicos
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
    double updateInterval = 0.02;
    double lastUpdate = 0;

    @Override
    public void init() {

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(
                        AprilTagGameDatabase.getBioBuzzTagLibrary()
                )
                .setLensIntrinsics(
                        545.584,
                        545.584,
                        350.588,
                        223.150
                )
                .setDrawAxes(false)
                .setDrawTagOutline(false)
                .build();

        VisionPortal.Builder builder =
                new VisionPortal.Builder()
                        .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                        .setCameraResolution(new Size(640, 480));

        if (USE_WEBCAM) {

            builder.setCamera(
                    hardwareMap.get(WebcamName.class, "Webcam 1")
            );

        } else {

            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

        // Nuevo nombre del dispositivo
        turret = hardwareMap.get(Servo.class, "turret");

        position = 0.50;
        turret.setPosition(position);
    }

    @Override
    public void loop() {

        List<AprilTagDetection> detections =
                aprilTag.getDetections();

        AprilTagClusterDetection target = null;

        // Buscar exclusivamente Blue Scoring
        for (AprilTagDetection detection : detections) {

            if (detection instanceof AprilTagClusterDetection
                    && detection.ftcPose != null) {

                AprilTagClusterDetection cluster =
                        (AprilTagClusterDetection) detection;

                if (cluster.metadata != null) {

                    String name =
                            cluster.metadata.name.toUpperCase(Locale.ROOT);

                    if (name.contains("BLUE SCORING")) {

                        target = cluster;
                        break;
                    }
                }
            }
        }

        if (target != null) {

            // Bearing original
            bearing = target.ftcPose.bearing;

            // Corregir referencia del centro
            correctedBearing = bearing - centerOffset;

            // Filtrado exponencial
            if (firstDetection) {

                filteredBearing = correctedBearing;
                firstDetection = false;

            } else {

                filteredBearing =
                        0.6 * filteredBearing
                                + 0.4 * correctedBearing;
            }

            // Actualización del controlador
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

                    // Control proporcional adaptativo
                    if (absError > 15) {

                        adaptiveKp = 0.60;
                        maxStepNow = 0.0012;

                    } else if (absError > 7) {

                        adaptiveKp = 0.40;
                        maxStepNow = 0.00065;

                    } else if (absError > 3) {

                        adaptiveKp = 0.18;
                        maxStepNow = 0.00020;

                    } else {

                        adaptiveKp = 0.06;
                        maxStepNow = 0.00008;
                    }

                    // Corrección proporcional
                    double correction =
                            (filteredBearing / sensitivity)
                                    * adaptiveKp
                                    * gearRatio;

                    // Limitar movimiento
                    correction = Math.max(
                            -maxStepNow,
                            Math.min(maxStepNow, correction)
                    );

                    // IMPORTANTE:
                    // Se invierte el sentido debido a los engranajes.
                    position += correction;

                    // Límites mecánicos
                    position = Math.max(
                            minPosition,
                            Math.min(maxPosition, position)
                    );

                    // Aplicar movimiento al turret
                    turret.setPosition(position);
                }
            }

            // TELEMETRY
            telemetry.addLine(
                    "=== APRILTAG CAMERA CHASIS REV1 ==="
            );

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
                    "Relación engranajes",
                    "%.2f:1", gearRatio
            );

            telemetry.addData(
                    "Corrigiendo",
                    correcting
            );

            telemetry.addData(
                    "Position turret",
                    "%.5f", position
            );

        } else {

            // Conservar posición al perder el objetivo
            firstDetection = true;
            correcting = false;

            telemetry.addLine(
                    "BLUE SCORING NO DETECTADO"
            );

            telemetry.addData(
                    "Position turret conservada",
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