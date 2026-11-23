package org.firstinspires.ftc.teamcode.HELP.Tests;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import java.util.List;
import java.util.Locale;

@Disabled
@TeleOp(name = "AprilTag_Camera_REV13")
public class AprilTag_Camera_REV13 extends OpMode {

    private Servo servo_pos;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final boolean USE_WEBCAM = true;

    // ==========================================
    // SERVO
    // ==========================================

    double position = 0.50;

    double minPosition = 0.44;
    double maxPosition = 0.63;

    double centerOffset = 8.0;
    double sensitivity = 490.0;

    // ==========================================
    // BEARING
    // ==========================================

    double bearing = 0;
    double correctedBearing = 0;
    double filteredBearing = 0;

    boolean correcting = false;
    boolean hadTarget = false;

    // ==========================================
    // CONTROL
    // ==========================================

    double updateInterval = 0.02;
    double lastUpdate = 0;

    // ==========================================
    // RECUPERACION
    // ==========================================

    boolean everDetected = false;
    boolean searching = false;

    double searchCenterPosition = 0.50;

    double searchMin;
    double searchMax;

    double searchDirection = 1.0;

    double searchStartTime = 0;
    double lastSearchUpdate = 0;

    // Busqueda mas suave
    double searchStep = 0.00018;
    double searchRange = 0.025;

    // ==========================================
    // INIT
    // ==========================================

    @Override
    public void init() {

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

        aprilTag.setDecimation(2.0f);

        VisionPortal.Builder builder =
                new VisionPortal.Builder()

                        .setStreamFormat(VisionPortal.StreamFormat.YUY2)

                        .setCameraResolution(
                                new Size(1280, 800)
                        );

        if (USE_WEBCAM) {

            builder.setCamera(
                    hardwareMap.get(
                            WebcamName.class,
                            "Webcam 1"
                    )
            );
        }

        builder.addProcessor(aprilTag);

        visionPortal = builder.build();

        servo_pos = hardwareMap.get(
                Servo.class,
                "servo"
        );

        position = 0.50;

        servo_pos.setPosition(position);
    }

    // ==========================================
    // LOOP
    // ==========================================

    @Override
    public void loop() {

        double now = getRuntime();

        List<AprilTagDetection> detections =
                aprilTag.getDetections();

        AprilTagClusterDetection target = null;

        // BUSCAR BLUE SCORING
        for (AprilTagDetection detection : detections) {

            if (detection instanceof AprilTagClusterDetection
                    && detection.ftcPose != null) {

                AprilTagClusterDetection cluster =
                        (AprilTagClusterDetection) detection;

                if (cluster.metadata != null) {

                    String name =
                            cluster.metadata.name.toUpperCase(
                                    Locale.ROOT
                            );

                    if (name.contains("BLUE SCORING")) {

                        target = cluster;
                        break;
                    }
                }
            }
        }

        // ======================================
        // DETECTADO
        // ======================================

        if (target != null) {

            bearing = target.ftcPose.bearing;

            correctedBearing =
                    bearing - centerOffset;

            // FILTRADO SUAVE
            if (!hadTarget) {

                filteredBearing = correctedBearing;

                correcting = false;

            } else {

                filteredBearing =
                        0.60 * filteredBearing
                                + 0.40 * correctedBearing;
            }

            hadTarget = true;
            everDetected = true;
            searching = false;

            // CONTROL
            if (now - lastUpdate >= updateInterval) {

                lastUpdate = now;

                double absError =
                        Math.abs(filteredBearing);

                // HISTÉRESIS
                if (!correcting && absError > 1.5) {
                    correcting = true;
                }

                if (correcting && absError < 0.7) {
                    correcting = false;
                }

                if (correcting) {

                    double adaptiveKp;
                    double maxStepNow;

                    // CONTROL MENOS AGRESIVO
                    if (absError > 15) {

                        adaptiveKp = 0.68;
                        maxStepNow = 0.0013;

                    } else if (absError > 7) {

                        adaptiveKp = 0.42;
                        maxStepNow = 0.00070;

                    } else if (absError > 3) {

                        adaptiveKp = 0.18;
                        maxStepNow = 0.00020;

                    } else {

                        adaptiveKp = 0.06;
                        maxStepNow = 0.00008;
                    }

                    double correction =
                            (filteredBearing / sensitivity)
                                    * adaptiveKp;

                    correction = Math.max(
                            -maxStepNow,
                            Math.min(maxStepNow, correction)
                    );

                    position -= correction;

                    position = Math.max(
                            minPosition,
                            Math.min(maxPosition, position)
                    );

                    servo_pos.setPosition(position);
                }
            }

            // TELEMETRY
            telemetry.addLine("=== REV13 TRACKING ===");

            telemetry.addData(
                    "Estado",
                    "BLUE SCORING DETECTADO"
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

        }

        // ======================================
        // PERDIDA DE DETECCION
        // ======================================

        else {

            hadTarget = false;
            correcting = false;

            if (everDetected && !searching) {

                searching = true;

                searchCenterPosition = position;

                searchMin = Math.max(
                        minPosition,
                        searchCenterPosition - searchRange
                );

                searchMax = Math.min(
                        maxPosition,
                        searchCenterPosition + searchRange
                );

                searchDirection = 1.0;

                searchStartTime = now;
                lastSearchUpdate = now;
            }

            if (searching && now - searchStartTime > 0.30) {

                if (now - lastSearchUpdate >= 0.05) {

                    lastSearchUpdate = now;

                    position +=
                            searchDirection * searchStep;

                    if (position >= searchMax) {

                        position = searchMax;
                        searchDirection = -1.0;

                    } else if (position <= searchMin) {

                        position = searchMin;
                        searchDirection = 1.0;
                    }

                    position = Math.max(
                            minPosition,
                            Math.min(maxPosition, position)
                    );

                    servo_pos.setPosition(position);
                }
            }

            telemetry.addLine("=== REV13 SEARCH ===");

            telemetry.addData(
                    "Estado",
                    searching
                            ? "BUSCANDO BLUE SCORING"
                            : "ESPERANDO DETECCION"
            );

            telemetry.addData(
                    "Position servo",
                    "%.5f", position
            );
        }

        telemetry.update();
    }

    // ==========================================
    // STOP
    // ==========================================

    @Override
    public void stop() {

        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}