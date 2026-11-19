package org.firstinspires.ftc.teamcode.TELEOP.TeleOp_Tests;


import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TELEOP.Mecanism_Tests.Mecanism_REV1;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

/*
 DRIVE [JOYSTICKS]+++++++
 * SLOW MODE [L_TRIGGER] ---> HOLD
 * INVERT MODE [L_BUMPER] ---> HOLD
 * INTAKE [R_TRIGGER] ---> HOLD
 * CAMERA OFF [B] ---> ONCE
 * CAMERA FRONT [D-UP] ---> ONCE
 * CAMERA RIGHT [D-RIGHT] ---> ONCE
 * CAMERA LEFT [D-LEFT] ---> ONCE
 * CAMERA REAR [D-REAR] ---> ONCE

 * =====GPAD 2======
 * SHOOTER POWER ON [A] ---> ONCE
 * SHOOTER POWER OFF [B] ---> ONCE
 * SHOOTER POWER +0.01 [D_UP] ---> ONCE
 * SHOOTER POWER -0.01 [D-DOWN] ---> ONCE
 * SHOOTER ANGLE +1 [RB] ---> ONCE
 * SHOOTER ANGLE -1 [LB] ---> ONCE
 * SHOOTER SELECTOR POLLEN [D-LEFT] ---> ONCE
 * SHOOTER SELECTOR NECTAR [D-RIGHT] ---> ONCE
 */
@Disabled
//@Configurable
@TeleOp(name = "TeleOp_Auto_Heading_REV5")
public class TeleOp_REV5 extends OpMode {
    boolean camera = true;

    //TL: servo_angle:
    boolean servo_angle_RBflag;
    boolean servo_angle_LBflag;
    double servo_angle_pos = 0.73;

    //TL: servo_selector:
    boolean servo_selector_RBflag;
    boolean servo_selector_LBflag;
    final double nectar = 0.79;
    final double pollen = 0.54;
    double servo_selector_pos = pollen;

    //TL: shooter:
    boolean shooter_RBflag;
    boolean shooter_LBflag;
    double shooter_power;

    //TL: AREA SHOOTING
    char area_disparo;











    //April Tag . . . . . . . . ..
    private Servo servo_pos;
    double incremento = 0.05;
    double position = 0.5;

    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    //TL: Turret positions
    double front = 0.43; //180 degrees
    double right = 0.31; //90 degrees
    double rear = 0.67; //0 degrees
    double left = 0.55; //270 degrees


    Mecanism_REV1 mecanism = new Mecanism_REV1();

    private Follower follower;
    //private TelemetryManager telemetryM;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose startPose = poseFactory.of(72, 72, 0);


    //TL:---------------------------HEADING HOLD--------------------------------

    // Ángulo que actualmente queremos mantener
    private double targetHeading;

    // Indica que el driver acaba de terminar de girar
    private boolean wasTurning = false;

    // Indica que estamos esperando a que el robot se estabilice
    private boolean capturingHeading = false;

    // Momento en el que el driver soltó el joystick
    private long headingReleaseTime;


    //TL:-------------------------------PID-------------------------------------
    private double headingIntegral = 0;
    private double previousHeadingError = 0;
    private long previousTime;


    //TL: AJUSTES DESDE PANELS
    public static double KP = 0.79;
    public static double KI = 0.0;
    public static double KD = 0.0025;

    // Zona muerta del joystick derecho
    public static double TURN_DEADBAND = 0.05;

    // Tiempo que esperamos después de soltar
    // el joystick antes de capturar el nuevo heading.
    public static double HEADING_CAPTURE_DELAY = 0.20;

    // Slow Mode
    public static double SLOW_MODE = 0.30;

    //TL: Calculo parabolico
    double axis_x;

    @Override
    public void init() {

        //April Tag. . . . . . . . . . . . . . . . . . . . . . . . .

        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(1432.032, 1432.032, 997.085, 535.891)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();


        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setCameraResolution(new Size(1280,800));

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }


        builder.addProcessor(aprilTag);
        visionPortal = builder.build();


        /*servo_pos = hardwareMap.get(Servo.class, "servo");
        servo_pos.setPosition(0.5);*/

        telemetry.addLine("Listo para detectar AprilTags y mostrar centro X/Y");
        //telemetry.update();

        mecanism.initAll(hardwareMap);

        //TL: servo_angle position:
        mecanism.servo_angle(0.73);

        //TL: servo_selector

        // Crear Follower
        follower = Constants.create(hardwareMap);

        // Posición inicial
        follower.setPose(startPose);

        // Telemetría de Panels
        //telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Actualizar localización
        //follower.update();

        // El heading inicial será nuestro primer objetivo
        targetHeading = follower.pose().heading();

        previousTime = System.nanoTime();

        telemetry.addLine("TeleOp listo");
        telemetry.addData(
                "Starting Heading",
                Math.toDegrees(targetHeading)
        );
        follower.update();
        telemetry.update();
    }


    @Override
    public void start() {

        // Activar control TeleOp de Pedro
        //follower.startTeleopDrive();

        // El heading donde comienza el partido
        // será el primer heading que debemos mantener
        targetHeading = follower.pose().heading();

        // Reiniciar PID
        headingIntegral = 0;
        previousHeadingError = 0;

        previousTime = System.nanoTime();

        wasTurning = false;
        capturingHeading = false;
    }


    @Override
    public void loop() {
        //TL: CONTROLES DEL DRIVER:
        double forward = -gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double driverTurn = -gamepad1.right_stick_x;

        //TL: servo_angle:
        mecanism.servo_angle(servo_angle_pos);
        boolean servo_angle_currentRB = gamepad2.right_bumper;
        boolean servo_angle_currentLB = gamepad2.left_bumper;

        if (servo_angle_currentRB && !servo_angle_RBflag && servo_angle_pos < 1.0){
            servo_angle_pos = servo_angle_pos + 0.01;
        }
        if (servo_angle_currentLB && !servo_angle_LBflag  && servo_angle_pos > 0.0){
            servo_angle_pos = servo_angle_pos - 0.01;
        }


        servo_angle_RBflag = servo_angle_currentRB;
        servo_angle_LBflag = servo_angle_currentLB;

        telemetry.addData("Servo angle pos: ", servo_angle_pos);




        //TL: servo_selector:
        /*mecanism.servo_selector(servo_selector_pos);
        boolean servo_selector_currentRB = gamepad2.dpad_right;
        boolean servo_selector_currentLB = gamepad2.dpad_left;

        if (servo_selector_currentRB && !servo_selector_RBflag && servo_selector_pos < 1.0){
            servo_selector_pos = servo_selector_pos + 0.01;
        }
        if (servo_selector_currentLB && !servo_selector_LBflag  && servo_selector_pos > 0.0){
            servo_selector_pos = servo_selector_pos - 0.01;
        }


        servo_selector_RBflag = servo_selector_currentRB;
        servo_selector_LBflag = servo_selector_currentLB;*/

        if (gamepad2.dpad_left) {
            servo_selector_pos = pollen;
            mecanism.servo_selector(servo_selector_pos);
            telemetry.addLine("POLLEN");
        }
        if (gamepad2.dpad_right) {
            servo_selector_pos = nectar;
            mecanism.servo_selector(servo_selector_pos);
            telemetry.addLine("NECTAR");
        }

        //telemetry.addData("Servo selector pos: ", servo_selector_pos);



        //TL: SHOOTER POWER
        mecanism.shooter(shooter_power);
        boolean shooter_power_currentRB = gamepad2.dpad_up;
        boolean shooter_power_currentLB = gamepad2.dpad_down;

        if (shooter_power_currentRB && !shooter_RBflag && shooter_power < 1.0){
            shooter_power = shooter_power + 0.01;
        }
        if (shooter_power_currentLB && !shooter_LBflag  && shooter_power > -1.0){
            shooter_power = shooter_power - 0.01;
        }


        shooter_RBflag = shooter_power_currentRB;
        shooter_LBflag = shooter_power_currentLB;

        telemetry.addData("Shooter power: ", shooter_power);

        //TL: -----------------------MODO INVERTIDO--------------------------
        /*
         * Cuando LB está presionado:
         *
         *      Adelante  → Atrás
         *      Atrás     → Adelante
         *      Izquierda → Derecha
         *      Derecha   → Izquierda
         *      Giro      → Giro contrario
         *
         * IMPORTANTE:
         * Aquí NO invertimos headingCorrection.
         * El Heading PID debe seguir corrigiendo
         * en la dirección física correcta.
         */

        if (gamepad1.left_bumper) {
            forward = -forward;
            lateral = -lateral;
            driverTurn = -driverTurn;
        }


        //TL: HEADING ACTUAL:
        double currentHeading = follower.pose().heading();

        double chasis_x_pos = follower.pose().x();
        double chasis_y_pos = follower.pose().y();

        //TL: ¿EL DRIVER ESTÁ GIRANDO?
        boolean driverTurning =
                Math.abs(driverTurn) > TURN_DEADBAND;

        double headingCorrection = 0;

        //TL: 1. DRIVER ESTÁ GIRANDO:
        if (driverTurning) {
            /*
             * EL DRIVER TIENE CONTROL TOTAL.
             *
             * El Heading Hold queda completamente desactivado.
             */

            headingCorrection = 0;

            wasTurning = true;
            capturingHeading = false;


            // Reiniciar PID
            headingIntegral = 0;
            previousHeadingError = 0;

            previousTime = System.nanoTime();
        }


        //TL: 2. DRIVER ACABA DE SOLTAR
        else if (wasTurning && !capturingHeading) {
            /*
             * El driver acaba de soltar el joystick.
             *
             * NO capturamos el heading inmediatamente.
             *
             * Primero dejamos que el robot termine de
             * girar por su propia inercia.
             */

            capturingHeading = true;

            headingReleaseTime = System.nanoTime();

            // PID todavía apagado
            headingCorrection = 0;

            headingIntegral = 0;
            previousHeadingError = 0;

            previousTime = System.nanoTime();
        }


        //TL: 3. ESPERANDO A QUE EL ROBOT SE ESTABILICE
        else if (capturingHeading) {

            long currentTime = System.nanoTime();

            double elapsedTime =
                    (currentTime - headingReleaseTime)
                            / 1_000_000_000.0;


            /*
             * Durante este pequeño período:
             *
             * - No corregimos
             * - No intentamos regresar
             * - Dejamos que el robot termine de girar
             */

            headingCorrection = 0;


            //TL: YA PASÓ EL TIEMPO DE ESTABILIZACIÓN

            if (elapsedTime >= HEADING_CAPTURE_DELAY) {

                /*
                 * Volvemos a leer el heading ACTUAL.
                 *
                 * Ese será el nuevo objetivo.
                 */

                currentHeading =
                        follower.pose().heading();

                targetHeading = currentHeading;


                // Reiniciar PID
                headingIntegral = 0;
                previousHeadingError = 0;

                previousTime = System.nanoTime();


                // Ya capturamos el nuevo heading
                capturingHeading = false;
                wasTurning = false;
            }
        }


        //TL: 4. HEADING HOLD NORMAL
        else {

            /*
             * Aquí el driver NO está girando
             * y ya tenemos un target establecido.
             */

            double headingError =
                    normalizeAngle(
                            targetHeading - currentHeading
                    );


            // Tiempo
            long currentTime = System.nanoTime();

            double deltaTime =
                    (currentTime - previousTime)
                            / 1_000_000_000.0;

            previousTime = currentTime;


            if (deltaTime <= 0) {
                deltaTime = 0.001;
            }


            //TL: P

            double proportional =
                    KP * headingError;


            //TL: I

            headingIntegral +=
                    headingError * deltaTime;

            double integral =
                    KI * headingIntegral;


            //TL: D

            double derivative =
                    (headingError - previousHeadingError)
                            / deltaTime;

            double derivativeTerm =
                    KD * derivative;


            previousHeadingError =
                    headingError;


            //TL: PID

            headingCorrection =
                    proportional
                            + integral
                            + derivativeTerm;


            // Limitar corrección
            headingCorrection =
                    Math.max(
                            -1.0,
                            Math.min(
                                    1.0,
                                    headingCorrection
                            )
                    );
        }


        //TL: GIRO FINAL
        double finalTurn;


        if (driverTurning) {

            /*
             * Driver manda directamente.
             *
             * Si estamos en modo invertido,
             * driverTurn ya fue invertido arriba.
             */
            finalTurn = driverTurn;

        } else if (capturingHeading) {

            /*
             * Estamos esperando que el robot termine
             * de asentarse.
             *
             * NO corregimos.
             */
            finalTurn = 0;

        } else {

            /*
             * Heading Hold está activo.
             *
             * IMPORTANTE:
             * NO invertimos esta corrección.
             */
            finalTurn = headingCorrection;
        }


        //TL:---------------------------SLOW MODE----------------------------

        if (gamepad1.left_trigger > 0) {

            forward *= SLOW_MODE;
            lateral *= SLOW_MODE;
            finalTurn *= SLOW_MODE;
        }


        //TL: DRIVE

        follower.manual(
                forward,
                lateral,
                finalTurn
        );

        //TL:---------------------------MECANISMOS------------------------------
        if (gamepad1.right_trigger > 0.1) {
            mecanism.intake(-0.8);
        }
        else {
            mecanism.intake(0);
        }

        if (gamepad1.dpad_down) {
            mecanism.turret(rear);
        }
        if (gamepad1.dpad_right) {
            mecanism.turret(right);
        }
        if (gamepad1.dpad_up) {
            mecanism.turret(front);
        }
        if (gamepad1.dpad_left) {
            mecanism.turret(left);
        }

        if (gamepad1.b) {
            camera = false;
        }
        if (gamepad1.a) {
            camera = true;
        }

        if (gamepad2.a) {
            shooter_power = 0.85;
        }
        if (gamepad2.b) {
            shooter_power = 0;
        }


        //TL: ---------------------------------------
        if ( ((chasis_x_pos > 55) && (chasis_x_pos < 82)) && ((chasis_y_pos >= 72) && (chasis_y_pos < 89))) {
            area_disparo = 'C';
        }
        else if ( ((chasis_x_pos > 84) && (chasis_x_pos < 103)) && ((chasis_y_pos >= 72) && (chasis_y_pos < 89))) {
            area_disparo = 'B';
        }
        else if ( ((chasis_x_pos > 39) && (chasis_x_pos < 57)) && ((chasis_y_pos >= 72) && (chasis_y_pos < 89))) {
            area_disparo = 'D';
        }
        else if ( ((chasis_x_pos > 55) && (chasis_x_pos < 82)) && ((chasis_y_pos > 90) && (chasis_y_pos < 107))) {
            area_disparo = 'A';
        }
        else {
            area_disparo = '0';
        }


        //TL: PANELS:

        /*telemetryM.debug(
                "Heading Hold activo cuando el driver no gira."
        );

        telemetryM.addData(
                "Current Heading",
                Math.toDegrees(currentHeading)
        );

        telemetryM.addData(
                "Target Heading",
                Math.toDegrees(targetHeading)
        );

        telemetryM.addData(
                "Heading Error",
                Math.toDegrees(
                        normalizeAngle(
                                targetHeading - currentHeading
                        )
                )
        );

        telemetryM.addData(
                "Driver Turn",
                driverTurn
        );

        telemetryM.addData(
                "Heading Correction",
                headingCorrection
        );

        telemetryM.addData(
                "Heading Hold",
                !driverTurning && !capturingHeading
        );

        telemetryM.addData(
                "Capturing Heading",
                capturingHeading
        );

        telemetryM.addData(
                "Capture Delay",
                HEADING_CAPTURE_DELAY
        );

        telemetryM.addData(
                "Inverted Mode",
                gamepad1.left_bumper
        );

        telemetryM.addData(
                "Slow Mode",
                gamepad1.left_trigger > 0
        );

        telemetryM.update(telemetry);*/


        //TL:------------------------------TELEMETRIA-------------------------------

        telemetry.addData("Chasis Heading", Math.toDegrees(currentHeading));
        telemetry.addData("Chasis X Pose: ", chasis_x_pos);
        telemetry.addData("Chasis Y Pose: ", chasis_y_pos);
        telemetry.addData("Area de disparo: ", area_disparo);
        telemetry.addLine("");

        //telemetry.addData("Target", Math.toDegrees(targetHeading));

        /*telemetry.addData(
                "Error",
                Math.toDegrees(
                        normalizeAngle(
                                targetHeading - currentHeading
                        )
                )
        );*/

        telemetry.addData(
                "Heading Hold",
                !driverTurning && !capturingHeading
        );

        telemetry.addData(
                "Inverted Mode",
                gamepad1.left_bumper
        );

        if (camera == true) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection instanceof AprilTagSingleDetection) {
                    AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;

                    if (singleDet.metadata != null) {
                        telemetry.addLine(String.format("\n==== (ID %d) %s", singleDet.id, singleDet.metadata.name));
                        telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                        telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                        telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                    } else {
                        telemetry.addLine(String.format("\n==== (ID %d) Unknown", singleDet.id));
                        telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", singleDet.center.x, singleDet.center.y));
                    }
                } else {
                    AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                    telemetry.addLine(String.format("\n==== Tag Cluster (%s)", clusterDet.metadata.name));
                    telemetry.addLine(String.format("Percent tags found: %d", clusterDet.percentClusterFound));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                }


                double errorX = detection.ftcPose.bearing;
                double ajuste = errorX * 0.000055;

                if (detection.ftcPose.bearing > 10) {
                    position = position + 0.0004;
                    mecanism.turret(position);
                    //servo_pos.setPosition(position);   //IZQUIERDA
                }

                if (detection.ftcPose.bearing < -10) {
                    position = position - 0.0004; //0.0008
                    mecanism.turret(position);
                    //servo_pos.setPosition(position);   //DERECHA
                }

                axis_x = Math.cos(45) * (detection.ftcPose.y * 2.54);


                //servo_pos.setPosition(position);

                //telemetry.addData("Position", servo_pos.getPosition());

            }   // end for() loop
        }


        // Add "key" information to telemetry
        telemetry.addData("Axis x: ", axis_x);
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

        follower.update();
        telemetry.update();
    }


    //TL: NORMALIZAR ÁNGULO

    private double normalizeAngle(double angle) {

        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }

        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }

        return angle;
    }
}