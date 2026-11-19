package org.firstinspires.ftc.teamcode.TELEOP.Mecanism_Tests;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**TL: PORTS
 *>>Control Hub:
 * motores:
 *  ("rightFront")  //0
 *  ("rightRear")    //2
 *  ("leftRear")  //3
 *  ("leftFront")    //1
 * Servos:
 *  ("turret") //0
 *  ("servo_angle") //1
 *  ("servo_selector") //2
 *
 * I2C:
 *  pinpoint = 1

 *>>Expansion Hub;
 * motores:
 *  intake = 0
 * servos:
 *
 */

public class Mecanism_REV1 {
    // Tl:========= INTAKE =========
    public DcMotor intake;

    // TL: -------- TURRET ---------
    private Servo turret;

    // TL: SERVOS TURRET
    private Servo servo_angle;

    private Servo servo_selector;


    public DcMotor shooter;

    //TL: ============= INIT =================
    public void initAll(HardwareMap hwMap){
        intake = hwMap.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turret = hwMap.get(Servo.class, "turret");

        servo_angle = hwMap.get(Servo.class, "servo_angle");

        servo_selector = hwMap.get(Servo.class, "servo_selector");

        shooter = hwMap.get(DcMotor.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //TL: ============== INTAKE ==============
    public void intake(double pow) {
        intake.setPower(pow);
    }

    public void turret(double position) {
        turret.setPosition(position);
    }

    public void servo_angle(double position) {
        //MAX --> 0.8
        //MIN --> 0.73
        servo_angle.setPosition(position);
    }

    public void servo_selector(double position) {
        //Nectar --> 0.79
        //Pollen --> 0.54
        servo_selector.setPosition(position);
    }

    public void shooter(double pow) {
        shooter.setPower(pow);
    }
}