package org.firstinspires.ftc.teamcode.HELP.Fixed;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo_Position", group = "test")
public class Servo_Position extends OpMode {
    boolean RBflag;
    boolean LBflag;
    boolean leftflag;
    boolean rightflag;
    Servo servo;
    double pos;
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");
    }

    @Override
    public void loop() {
        servo.setPosition(pos);
        boolean currentRB = gamepad1.right_bumper;
        boolean currentLB = gamepad1.left_bumper;
        boolean currentleft = gamepad1.dpad_left;
        boolean currentright = gamepad1.dpad_right;

        if (currentRB && !RBflag && pos < 1.0){
            pos = pos + 0.01;
        }
        if (currentLB && !LBflag  && pos > 0.0){
            pos = pos - 0.01;
        }

        if (currentleft && !leftflag && pos > 0.0){
            pos = pos - 0.001;
        }
        if (currentright && !rightflag  && pos < 1.0){
            pos = pos + 0.001;
        }


        if (gamepad1.a){pos = 0.0;}
        if (gamepad1.b){pos = 0.5;}
        if (gamepad1.y){pos = 1.0;}
        RBflag = currentRB;
        LBflag = currentLB;

        leftflag = currentleft;
        rightflag = currentright;

        telemetry.addData("POS: ", pos);
    }
}