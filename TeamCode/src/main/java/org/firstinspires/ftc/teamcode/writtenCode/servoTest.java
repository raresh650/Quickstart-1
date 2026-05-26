package org.firstinspires.ftc.teamcode.writtenCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@TeleOp(name = "servoTest", group = "TeleOp")
public class servoTest extends OpMode {

    // Servos
    private Servo servo1;
    //private Servo servo2;

    // Positions (public static so other classes can read/use them)
    public static double POS1 = 0.5;

    @Override
    public void init() {
        // Map from the hardware configuration
        servo1 = hardwareMap.get(Servo.class, "stopper");
        //servo2 = hardwareMap.get(Servo.class, "hood1");

        // Set both servos to a starting position
        servo1.setPosition(POS1);
        //servo2.setPosition(1-POS1);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        servo1.setPosition(POS1);
        //servo2.setPosition(1-POS1);
        telemetry.addData("Servo1", servo1.getPosition());
        //telemetry.addData("Servo2", servo2.getPosition());
        telemetry.update();
    }
}