package org.firstinspires.ftc.teamcode.writtenCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "motorTest", group = "TeleOp")
public class motorTest extends OpMode {

    // Motors
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotorEx motor3;
    private DcMotorEx motor4;
    private Servo servo1;
    public static double POS1 = 0;

    @Override
    public void init() {
        // Map from the hardware configuration
        //motor1 = hardwareMap.get(DcMotor.class, "flywheel2");
        motor2 = hardwareMap.get(DcMotor.class, "flywheel1");
        //motor3 = hardwareMap.get(DcMotorEx.class, "transfer");
        //motor4 = hardwareMap.get(DcMotorEx.class, "intake");
        //servo1 = hardwareMap.get(Servo.class, "stopper");
        //servo1.setPosition(POS1);

        // Optional: brake when power is zero
        //motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Run both motors at full power
        //motor1.setPower(1);
        motor2.setPower(1);
        //motor3.setPower(-1.0);
        //motor4.setPower(-1.0);
        //servo1.setPosition(POS1);

        //telemetry.addData("Motor1 Power", motor1.getPower());
        //telemetry.addData("Motor2 Power", motor2.getPower());

        // Amperage for motor 3 (transfer) and motor 4 (intake)
        telemetry.addData("Motor3 (transfer) Amps", motor3.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Motor4 (intake) Amps", motor4.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
}