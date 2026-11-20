package org.firstinspires.ftc.teamcode.teleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "HoodCalib")
public class HoodCalib extends OpMode {

    double pos = 0.5;
    Servo servo;


    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "hood");
    }

    @Override
    public void start() {
     servo.setPosition(pos);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) {
            pos += 0.01;
            servo.setPosition(pos);
        }

        if (gamepad1.dpadDownWasPressed()) {
            pos -= 0.01;
            servo.setPosition(pos);
        }


        // --- телеметрия ---
        telemetry.addData("ServoPos", pos);


    }

}