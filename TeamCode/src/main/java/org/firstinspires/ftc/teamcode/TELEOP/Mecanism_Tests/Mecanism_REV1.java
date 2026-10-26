package org.firstinspires.ftc.teamcode.TELEOP.Mecanism_Tests;

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
 *
 * I2C:
 *  pinpoint = 2

 *>>Expansion Hub;
 * motores:
 *  Intake = 0
 * servos:
 *
 */

public class Mecanism_REV1 {
    // Tl:========= INTAKE =========
    public DcMotor intake;

    // TL: -------- TURRET ---------
    private Servo turret;

    //TL: ============= INIT =================
    public void initAll(HardwareMap hwMap){
        intake = hwMap.get(DcMotor.class, "Intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turret = hwMap.get(Servo.class, "turret");
    }

    //TL: ============== INTAKE ==============
    public void intake(double pow) {
        intake.setPower(pow);
    }

    public void turret(double position) {
        turret.setPosition(position);
    }
}