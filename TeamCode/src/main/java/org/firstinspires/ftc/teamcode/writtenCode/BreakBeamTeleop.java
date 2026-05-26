package org.firstinspires.ftc.teamcode.writtenCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * BreakBeamTeleop
 *
 * Minimal teleop: reads a single break beam sensor and reports whether
 * the beam is broken or clear on the telemetry.
 *
 * A typical IR break beam reads HIGH when the beam is intact and LOW when
 * something is blocking it, so broken == !getState(). If yours reads
 * backwards, remove the "!" on the isBroken line.
 *
 * Config: the receiver's signal wire is set up as a Digital Device named
 * "beam1". The transmitter only needs power, so it gets no config entry.
 */
@TeleOp(name = "Break Beam Check", group = "Sensor")
public class BreakBeamTeleop extends OpMode {

    private DigitalChannel beam1;

    @Override
    public void init() {
        beam1 = hardwareMap.get(DigitalChannel.class, "beam1");
        beam1.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("Status", "Initialized - ready to check beam");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean beamBroken = !beam1.getState();

        telemetry.addData("Beam", beamBroken ? "BROKEN" : "clear");
        telemetry.update();
    }
}