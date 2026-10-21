package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test TeleOp", group = "Test")
public class TestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addLine("¡OpMode registrado correctamente!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Test TeleOp funcionando");
            telemetry.update();
        }
    }
}
