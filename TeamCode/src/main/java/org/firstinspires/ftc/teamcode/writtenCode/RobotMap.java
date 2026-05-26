package org.firstinspires.ftc.teamcode.writtenCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class   RobotMap {

    public DcMotorEx intake,transfer;

    public Servo hood1;
    public Servo hood2;
    public Servo turret1,turret2;
    public Servo stopper;

    public RobotMap(HardwareMap Init)
    {
        intake = Init.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        transfer = Init.get(DcMotorEx.class, "transfer");
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);



        hood1 = Init.get(Servo.class, "hood1");
        hood2 = Init.get(Servo.class, "hood2");
        turret1 = Init.get(Servo.class, "turret1");
        turret2 = Init.get(Servo.class, "turret2");
        stopper = Init.get(Servo.class, "stopper");
    }
}
