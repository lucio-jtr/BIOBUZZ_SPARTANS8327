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
@TeleOp(name = "AprilTag_Camera_REV4")
public class AprilTag_Camera_REV4 extends OpMode {

    private Servo servo_pos;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final boolean USE_WEBCAM = true;

    // Posicion inicial
    double position = 0.50;

    // Sensibilidad experimental
    double sensitivity = 490.0;

    // Zona muerta
    double deadband = 0.7;

    // Movimiento maximo por actualizacion
    double maxStep = 0.0008;

    // Intervalo de actualizacion
    double updateInterval = 0.04;

    // Limites del servo
    double minPosition = 0.44;
    double maxPosition = 0.63;

    // Bearing
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

        // Buscar primero el cluster
        for (AprilTagDetection detection : detections) {

            if (detection instanceof AprilTagClusterDetection
                    && detection.ftcPose != null) {

                target = detection;
                break;
            }
        }

        // Buscar tag individual si no hay cluster
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

            // Filtrado del bearing
            if (firstDetection) {

                filteredBearing = bearing;
                firstDetection = false;

            } else {

                filteredBearing =
                        0.7 * filteredBearing + 0.3 * bearing;
            }

            // Actualizacion controlada
            if (getRuntime() - lastUpdate >= updateInterval) {

                lastUpdate = getRuntime();

                double absError =
                        Math.abs(filteredBearing);

                if (absError > deadband) {

                    // Ganancia adaptativa
                    double adaptiveKp;

                    if (absError > 15) {
                        adaptiveKp = 0.45;
                    }
                    else if (absError > 7) {
                        adaptiveKp = 0.30;
                    }
                    else if (absError > 3) {
                        adaptiveKp = 0.18;
                    }
                    else {
                        adaptiveKp = 0.10;
                    }

                    // Calcular correccion
                    double correction =
                            (filteredBearing / sensitivity)
                                    * adaptiveKp;

                    // Limitar movimiento
                    correction = Math.max(
                            -maxStep,
                            Math.min(maxStep, correction)
                    );

                    // Corregir posicion
                    position -= correction;

                    // Limitar recorrido
                    position = Math.max(
                            minPosition,
                            Math.min(maxPosition, position)
                    );

                    servo_pos.setPosition(position);
                }
            }

            telemetry.addData("ESTADO", "TAG DETECTADO");
            telemetry.addData("BEARING", "%.2f", bearing);
            telemetry.addData("BEARING FILTRADO",
                    "%.2f", filteredBearing);
            telemetry.addData("ERROR ABSOLUTO",
                    "%.2f", Math.abs(filteredBearing));
            telemetry.addData("POSITION", "%.5f", position);
            telemetry.addData("SERVO", "%.5f",
                    servo_pos.getPosition());

        } else {

            // Conservar posicion cuando se pierde el tag
            firstDetection = true;

            telemetry.addData("ESTADO", "TAG PERDIDO");
            telemetry.addData("POSITION", "%.5f", position);
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