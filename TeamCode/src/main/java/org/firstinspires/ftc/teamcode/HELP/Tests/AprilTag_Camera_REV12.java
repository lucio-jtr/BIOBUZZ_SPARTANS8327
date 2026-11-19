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

@TeleOp(name = "AprilTag_Camera_REV12")
public class AprilTag_Camera_REV12 extends OpMode {

    private Servo servo_pos;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final boolean USE_WEBCAM = true;

    // ==========================================
    // CONFIGURACION DEL SERVO
    // ==========================================

    double position = 0.50;

    double minPosition = 0.44;
    double maxPosition = 0.63;

    // Calibracion del centro de la CELL
    double centerOffset = 8.0;

    // Sensibilidad experimental
    double sensitivity = 490.0;

    // ==========================================
    // VARIABLES DE SEGUIMIENTO
    // ==========================================

    double bearing = 0;
    double correctedBearing = 0;
    double filteredBearing = 0;

    boolean correcting = false;
    boolean hadTarget = false;

    // ==========================================
    // VELOCIDAD DEL CONTROLADOR
    // ==========================================

    double updateInterval = 0.02;
    double lastUpdate = 0;

    // ==========================================
    // RECUPERACION DEL APRILTAG
    // ==========================================

    boolean everDetected = false;
    boolean searching = false;

    double searchCenterPosition = 0.50;

    double searchMin;
    double searchMax;

    double searchDirection = 1.0;

    double searchStartTime = 0;
    double lastSearchUpdate = 0;

    // Velocidad de busqueda
    double searchStep = 0.00035;

    // Limites de busqueda alrededor
    // de la ultima posicion conocida
    double searchRange = 0.030;

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

        // Mayor decimacion = menos procesamiento
        // y potencialmente mayor velocidad
        aprilTag.setDecimation(2.0f);

        VisionPortal.Builder builder =
                new VisionPortal.Builder()

                        .setStreamFormat(
                                VisionPortal.StreamFormat.YUY2
                        )

                        // Conservamos la resolucion calibrada
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

        } else {

            builder.setCamera(
                    BuiltinCameraDirection.BACK
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

        telemetry.addLine("REV12 INICIADA");
        telemetry.addLine("Esperando Blue Scoring...");
        telemetry.update();
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

        // ======================================
        // BUSCAR BLUE SCORING
        // ======================================

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
        // SI ENCUENTRA EL APRILTAG
        // ======================================

        if (target != null) {

            // Bearing original
            bearing = target.ftcPose.bearing;

            // Corregir centro fisico de CELL
            correctedBearing =
                    bearing - centerOffset;

            // Filtrado mas rapido
            if (!hadTarget) {

                filteredBearing = correctedBearing;

                correcting = false;

            } else {

                filteredBearing =
                        0.45 * filteredBearing
                                + 0.55 * correctedBearing;
            }

            hadTarget = true;
            everDetected = true;
            searching = false;

            // ==================================
            // CONTROLADOR RAPIDO
            // ==================================

            if (now - lastUpdate >= updateInterval) {

                lastUpdate = now;

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

                    // Ganancia adaptativa
                    if (absError > 15) {

                        adaptiveKp = 0.80;
                        maxStepNow = 0.0018;

                    } else if (absError > 7) {

                        adaptiveKp = 0.50;
                        maxStepNow = 0.0009;

                    } else if (absError > 3) {

                        adaptiveKp = 0.18;
                        maxStepNow = 0.00020;

                    } else {

                        adaptiveKp = 0.06;
                        maxStepNow = 0.00008;
                    }

                    // Calcular correccion
                    double correction =
                            (filteredBearing / sensitivity)
                                    * adaptiveKp;

                    // Limitar movimiento
                    correction = Math.max(
                            -maxStepNow,
                            Math.min(maxStepNow, correction)
                    );

                    // Aplicar movimiento
                    position -= correction;

                    // Limites mecanicos
                    position = Math.max(
                            minPosition,
                            Math.min(maxPosition, position)
                    );

                    servo_pos.setPosition(position);
                }
            }

            // ==================================
            // TELEMETRY DE SEGUIMIENTO
            // ==================================

            telemetry.addLine("=== REV12 TRACKING ===");

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
                    "Buscando",
                    searching
            );

            telemetry.addData(
                    "Position servo",
                    "%.5f", position
            );

        }

        // ======================================
        // SI PIERDE BLUE SCORING
        // ======================================

        else {

            hadTarget = false;
            correcting = false;

            // Inicializar busqueda una sola vez
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

            // Esperar antes de comenzar a buscar
            if (searching && now - searchStartTime > 0.25) {

                if (now - lastSearchUpdate >= 0.04) {

                    lastSearchUpdate = now;

                    // Barrido controlado
                    position +=
                            searchDirection * searchStep;

                    // Cambiar direccion al alcanzar limites
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

            // ==================================
            // TELEMETRY DE RECUPERACION
            // ==================================

            telemetry.addLine("=== REV12 SEARCH ===");

            if (searching) {

                telemetry.addData(
                        "Estado",
                        "BUSCANDO BLUE SCORING"
                );

                telemetry.addData(
                        "Direccion",
                        searchDirection > 0
                                ? "DERECHA"
                                : "IZQUIERDA"
                );

                telemetry.addData(
                        "Rango inferior",
                        "%.5f", searchMin
                );

                telemetry.addData(
                        "Rango superior",
                        "%.5f", searchMax
                );

            } else {

                telemetry.addData(
                        "Estado",
                        "ESPERANDO PRIMERA DETECCION"
                );
            }

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