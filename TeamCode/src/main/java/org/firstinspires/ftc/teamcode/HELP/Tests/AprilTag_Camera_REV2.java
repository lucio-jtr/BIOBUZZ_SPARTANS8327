package org.firstinspires.ftc.teamcode.HELP.Tests;


import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.util.List;

@TeleOp(name = "AprilTag_Camera_REV2", group = "Tests")
public class AprilTag_Camera_REV2 extends OpMode {

    private Servo servo_pos;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Recorrido inicial limitado por seguridad.
    private static final double START_POS = 0.50;
    private static final double MIN_POS = 0.30; //0.48
    private static final double MAX_POS = 0.70; //0.52

    // Cambiar a -1.0 si sigue el objetivo en sentido contrario.
    private static final double DIRECTION = 1.0;

    // Ganancias conservadoras para servo posicional de 5 vueltas.
    private static final double KP = 0.00012;
    private static final double KD = 0.0;

    private static final double DEAD_BAND_DEG = 1.5;
    private static final double FILTER_ALPHA = 0.25;
    private static final double MAX_STEP = 0.0003;
    private static final double MAX_DERIVATIVE = 20.0;

    private double position = START_POS;
    private double filteredBearing = 0.0;
    private double previousBearing = 0.0;
    private boolean hadTarget = false;

    @Override
    public void init() {
        servo_pos = hardwareMap.get(Servo.class, "servo");
        servo_pos.setPosition(START_POS);

        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(
                        hardwareMap.get(WebcamName.class, "Webcam 1")
                )
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("AprilTag follow - SDK 12");
        telemetry.addLine("Prioridad: Cluster, despues Single");
        telemetry.addLine("Recorrido inicial limitado");
        telemetry.update();
    }

    @Override
    public void start() {
        position = START_POS;
        servo_pos.setPosition(position);
        hadTarget = false;
    }

    @Override
    public void loop() {
        List<AprilTagDetection> detections =
                aprilTag.getDetections();

        AprilTagDetection selected = null;
        String selectedType = "Ninguno";
        double bestBearingMagnitude = Double.POSITIVE_INFINITY;

        // PRIMERA PRIORIDAD: clusters.
        for (AprilTagDetection detection : detections) {
            if (!(detection instanceof AprilTagClusterDetection)) {
                continue;
            }

            if (detection.ftcPose == null) {
                continue;
            }

            double bearing = detection.ftcPose.bearing;
            double x = detection.ftcPose.x;
            double y = detection.ftcPose.y;

            if (!Double.isFinite(bearing)
                    || !Double.isFinite(x)
                    || !Double.isFinite(y)
                    || y <= 0) {
                continue;
            }

            if (Math.abs(bearing) < bestBearingMagnitude) {
                selected = detection;
                selectedType = "CLUSTER";
                bestBearingMagnitude = Math.abs(bearing);
            }
        }

        // SEGUNDA PRIORIDAD: tags individuales.
        // Solo se utilizan si no hay un cluster válido.
        if (selected == null) {
            for (AprilTagDetection detection : detections) {
                if (!(detection instanceof AprilTagSingleDetection)) {
                    continue;
                }

                if (detection.ftcPose == null) {
                    continue;
                }

                double bearing = detection.ftcPose.bearing;
                double x = detection.ftcPose.x;
                double y = detection.ftcPose.y;

                if (!Double.isFinite(bearing)
                        || !Double.isFinite(x)
                        || !Double.isFinite(y)
                        || y <= 0) {
                    continue;
                }

                if (Math.abs(bearing) < bestBearingMagnitude) {
                    selected = detection;
                    selectedType = "SINGLE";
                    bestBearingMagnitude = Math.abs(bearing);
                }
            }
        }

        // Sin objetivo válido: mantener la posición actual.
        if (selected == null) {
            hadTarget = false;

            telemetry.addData("Estado", "Sin pose valida");
            telemetry.addData("Detecciones", detections.size());
            telemetry.addData("Servo", "%.4f", position);
            telemetry.update();
            return;
        }

        double bearing = selected.ftcPose.bearing;
        double x = selected.ftcPose.x;
        double y = selected.ftcPose.y;

        // Filtrado temporal del bearing.
        if (!hadTarget) {
            filteredBearing = bearing;
            previousBearing = bearing;
            hadTarget = true;
        } else {
            filteredBearing += FILTER_ALPHA
                    * (bearing - filteredBearing);
        }

        // Derivada discreta: anticipación limitada.
        double derivative = filteredBearing - previousBearing;

        derivative = Range.clip(
                derivative,
                -MAX_DERIVATIVE,
                MAX_DERIVATIVE
        );

        previousBearing = filteredBearing;

        double step = 0.0;

        if (Math.abs(filteredBearing) > DEAD_BAND_DEG) {
            step = KP * filteredBearing + KD * derivative;

            step = Range.clip(step, -MAX_STEP, MAX_STEP);
            step *= DIRECTION;

            position += step;
        }

        position = Range.clip(position, MIN_POS, MAX_POS);

        // Solo una orden de posición por ciclo.
        servo_pos.setPosition(position);

        telemetry.addData("Estado", "SIGUIENDO");
        telemetry.addData("Tipo", selectedType);
        //telemetry.addData("ID", selected.id);
        telemetry.addData("Bearing", "%.2f deg", bearing);
        telemetry.addData("Bearing filtrado", "%.2f", filteredBearing);
        telemetry.addData("X", "%.2f in", x);
        telemetry.addData("Y", "%.2f in", y);

        if (selected instanceof AprilTagSingleDetection) {
            AprilTagSingleDetection single =
                    (AprilTagSingleDetection) selected;
            telemetry.addData("Centro X", "%.1f px", single.center.x);
        } else if (selected instanceof AprilTagClusterDetection) {
            AprilTagClusterDetection cluster =
                    (AprilTagClusterDetection) selected;
            telemetry.addData(
                    "Cluster",
                    "%s (%d%%)",
                    cluster.metadata != null
                            ? cluster.metadata.name : "sin metadata",
                    cluster.percentClusterFound
            );
        }

        telemetry.addData("Correccion", "%.5f", step);
        telemetry.addData("Posicion servo", "%.4f", position);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
