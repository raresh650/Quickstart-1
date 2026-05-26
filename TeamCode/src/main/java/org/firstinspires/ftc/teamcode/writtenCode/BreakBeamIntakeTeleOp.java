package org.firstinspires.ftc.teamcode.writtenCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Break Beam Intake TeleOp", group = "TeleOp")
public class BreakBeamIntakeTeleOp extends OpMode {

    // --- Hardware ---
    private DcMotor intake;
    private DcMotor transfer;
    private DigitalChannel breakBeam;

    // --- Tuning ---
    private static final double INTAKE_POWER = -1.0;    // intake run power
    private static final double DETECT_SECONDS = 1.0;   // how long the beam must stay broken

    // Most break beams read FALSE when the beam is broken (object present).
    // If yours is wired the opposite way, flip this to true.
    private static final boolean BROKEN_WHEN_TRUE = false;

    // --- State ---
    private final ElapsedTime beamTimer = new ElapsedTime();
    private boolean intakeLockedOff = false;  // true once an artifact has been confirmed for 1s

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        breakBeam = hardwareMap.get(DigitalChannel.class, "breakBeam");
        breakBeam.setMode(DigitalChannel.Mode.INPUT);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);
        transfer.setPower(0);

        telemetry.addLine("Initialized. Ready to run.");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean artifactDetected = isBeamBroken();

        if (artifactDetected) {
            // Beam is broken: if this is the first frame of detection the timer
            // is already running from the last clear; just check how long it's held.
            if (beamTimer.seconds() >= DETECT_SECONDS) {
                intakeLockedOff = true;
            }
        } else {
            // Beam clear: reset the timer so brief blips don't accumulate,
            // and re-allow the intake to run.
            beamTimer.reset();
            intakeLockedOff = false;
        }

        // Drive intake + transfer together. Hold A on gamepad1 to run them
        // (unless an artifact has locked the system off).
        double power;
        if (intakeLockedOff) {
            power = 0;
        } else if (gamepad1.a) {
            power = INTAKE_POWER;
        } else {
            power = 0;
        }
        intake.setPower(power);
        transfer.setPower(power);

        telemetry.addData("Beam broken", artifactDetected);
        telemetry.addData("Held (s)", "%.2f", beamTimer.seconds());
        telemetry.addData("Intake locked off", intakeLockedOff);
        telemetry.update();
    }

    /** Returns true when an artifact is breaking the beam. */
    private boolean isBeamBroken() {
        boolean raw = breakBeam.getState();   // true = beam intact on most sensors
        return BROKEN_WHEN_TRUE == raw;
    }

    @Override
    public void stop() {
        intake.setPower(0);
        transfer.setPower(0);
    }
}