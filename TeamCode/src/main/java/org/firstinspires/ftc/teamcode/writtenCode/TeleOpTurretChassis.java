package org.firstinspires.ftc.teamcode.writtenCode;

import static org.firstinspires.ftc.teamcode.writtenCode.controllers.IntakeController.IntakeStatus.OFF;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.writtenCode.controllers.HoodController;
import org.firstinspires.ftc.teamcode.writtenCode.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.writtenCode.controllers.PIDFController;
import org.firstinspires.ftc.teamcode.writtenCode.controllers.StopperController;
import org.firstinspires.ftc.teamcode.writtenCode.controllers.TransferController;
import org.firstinspires.ftc.teamcode.writtenCode.controllers.TurretController;

@Configurable
@Config
@TeleOp(name="TeleOpCode Blue (Turret + Drive + Intake)", group="Linear OpMode")
public class TeleOpTurretChassis extends LinearOpMode {

    // ============================================================
    // Chassis helpers
    // ============================================================
    public void setMotorRunningMode(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront,
                                    DcMotor rightBack, DcMotor.RunMode runningMode) {
        leftFront.setMode(runningMode);
        rightFront.setMode(runningMode);
        leftBack.setMode(runningMode);
        rightBack.setMode(runningMode);
    }

    public void setMotorZeroPowerBehaviour(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront,
                                           DcMotor rightBack, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        leftFront.setZeroPowerBehavior(zeroPowerBehavior);
        rightFront.setZeroPowerBehavior(zeroPowerBehavior);
        leftBack.setZeroPowerBehavior(zeroPowerBehavior);
        rightBack.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void robotCentricDrive(DcMotor leftFront, DcMotor leftBack,
                                  DcMotor rightFront, DcMotor rightBack,
                                  double leftTrigger, double rightTrigger, double rate) {

        double y = -gamepad2.left_stick_y;
        double x = (gamepad2.right_trigger - gamepad2.left_trigger) * 1.05;
        double rx = gamepad2.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower  = (y + x + rx) / denominator * rate;
        double leftBackPower   = (y - x + rx) / denominator * rate;
        double rightFrontPower = (y - x - rx) / denominator * rate;
        double rightBackPower  = (y + x - rx) / denominator * rate;

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    // ============================================================
    // State
    // ============================================================
    ElapsedTime GlobalTimer = new ElapsedTime();

    private Telemetry dashTelemetry;
    private Follower  follower;

    boolean turretOn = false;
    boolean treibile = false;   // true only when BOTH beams stay broken for delayIntake seconds

    // ---- Break beam sensors ----
    // Receiver signal wires configured as Digital Devices.
    // A typical IR break beam reads HIGH when intact, LOW when blocked,
    // so broken == !getState(). Flip the "!" if yours reads backwards.
    private DigitalChannel beam1;
    private DigitalChannel beam2;

    // Tracks how long beam1 has been continuously broken (for delayIntake auto-off)
    private final ElapsedTime beam1Timer = new ElapsedTime();

    // Tracks how long BOTH beams have been continuously broken (for treibile)
    private final ElapsedTime bothBeamsTimer = new ElapsedTime();

    // ---- Transfer jam protection ----
    public static double TRANSFER_AMP_LIMIT = 7.0;

    // ---- Flywheel PID + shooting ----
    private PIDFController controller;
    private DcMotorEx motor;   // flywheelMotorR
    private DcMotorEx motor1;  // flywheelMotorL

    private final ElapsedTime shootTime         = new ElapsedTime();
    private final ElapsedTime shootEndTime      = new ElapsedTime();
    private final ElapsedTime shootTransferTime = new ElapsedTime();

    boolean flywheelOn    = false;
    boolean shootTransfer = false;
    boolean shooting      = false;

    public static double delayShoot      = 0.4;  // spin-up time before feeding balls
    public static double shootEndCounter = 0.2;   // cooldown between shots

    // Seconds BOTH beams must stay continuously broken before treibile goes true
    // (also reused for the beam1 intake auto-off countdown)
    public static double delayIntake = 0.2;

    public static double SHOOT_VELOCITY = 1900;   // flywheel target (ticks/s), runs continuously
    public static double hoodTargetPos = 0.37;
    public static double targetVelocity, velocity;
    public static double P = 0.007, I = 0.01, kV = 0.000435, kS = 0.03, kD = 0;

    public static Pose startingPose = new Pose(72, 72, Math.toRadians(0));

    public static double TURRET_OFFSET_X = -2.052;
    public static double TURRET_OFFSET_Y =  0.0;

    public static double turretOffset = 0.0;

    public static double robot_pose_x, robot_pose_y, robot_angle,
            robot_velocity_magnitude, robot_velocity_angle;
    public static double goal_pose_x = 1, goal_pose_y = 143;

    public static double turret_field_x, turret_field_y;

    public static double bearingToGoal;
    public static double distanceToGoal;
    public static double turret_target_position = 0.5;
    public static double degreesToTurn;
    public double rate = 1;

    // Constants used by the launch math that feeds the turret velocity offset
    public static double goalHeight = 30;
    public static double theta      = -0.35;

    public static double hoodpos = 0.59;
    public static double shootest = 1200;

    // Turret physical limits
    // servo 0.5 == pointing straight BEHIND the robot. Travel is +/-180deg
    // from that center (360deg total).
    private static final double TURRET_MAX_DEGREES = 180.0;

    // ---- Turret servo mapping ----
    // 0.5   = init / half turn, facing the BACK of the robot
    // 0.075 = facing the FRONT (counter-clockwise limit)
    // 0.925 = facing the FRONT (clockwise limit)
    // Outside [0.075, 0.925] the cable tangles -> never command past it.
    public static double  TURRET_CENTER     = 0.5;    // back / half turn
    public static double  TURRET_MIN        = 0.075;  // CCW front limit
    public static double  TURRET_MAX        = 0.925;  // CW front limit
    public static double  TURRET_HALF_RANGE = 0.425;  // servo units for a 180° turn (0.5 - 0.075)
    public static boolean TURRET_MIRROR     = true;   // false if both servos spin the SAME direction

    @Override
    public void runOpMode() throws InterruptedException {

        dashTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        follower.update();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);   // <-- apply it here, in init
        follower.update();

        RobotMap robot = new RobotMap(hardwareMap);

        HoodController     hoodController     = new HoodController(robot);
        IntakeController   intakeController   = new IntakeController(robot);
        TransferController transferController = new TransferController(robot);
        StopperController  stopperController  = new StopperController(robot);
        TurretController   turretController   = new TurretController(robot);

        // ---- Break beams ----
        beam1 = hardwareMap.get(DigitalChannel.class, "beam1");
        beam2 = hardwareMap.get(DigitalChannel.class, "beam2");
        beam1.setMode(DigitalChannel.Mode.INPUT);
        beam2.setMode(DigitalChannel.Mode.INPUT);

        // ---- Flywheel ----
        motor  = hardwareMap.get(DcMotorEx.class, "flywheel1");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        controller = new PIDFController(P, I, kD, 0.0);

        // ---- Turret servos ----
        Servo turretServo1 = hardwareMap.get(Servo.class, "turret1");
        Servo turretServo2 = hardwareMap.get(Servo.class, "turret2");

        intakeController.update();
        transferController.update();
        stopperController.update();
        hoodController.update(hoodTargetPos);
        turretController.update(turret_target_position);

        // ---- Drive motors ----
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightBack  = hardwareMap.get(DcMotor.class, "rightBack");
        DcMotor leftBack   = hardwareMap.get(DcMotor.class, "leftBack");

        MotorConfigurationType mct1, mct2, mct3, mct4;
        mct1 = rightBack.getMotorType().clone();
        mct1.setAchieveableMaxRPMFraction(1.0);
        rightBack.setMotorType(mct1);

        mct2 = rightFront.getMotorType().clone();
        mct2.setAchieveableMaxRPMFraction(1.0);
        rightFront.setMotorType(mct2);

        mct3 = leftFront.getMotorType().clone();
        mct3.setAchieveableMaxRPMFraction(1.0);
        leftFront.setMotorType(mct3);

        mct4 = leftBack.getMotorType().clone();
        mct4.setAchieveableMaxRPMFraction(1.0);
        leftBack.setMotorType(mct4);

        setMotorRunningMode(leftFront, leftBack, rightFront, rightBack,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        setMotorZeroPowerBehaviour(leftFront, leftBack, rightFront, rightBack,
                DcMotor.ZeroPowerBehavior.BRAKE);

        Gamepad currentGamepad1  = new Gamepad();
        Gamepad currentGamepad2  = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();


        waitForStart();
        GlobalTimer.reset();

        while (opModeIsActive()) {

            if (isStopRequested()) return;

            follower.update();

            // ---- Break beam reads ----
            boolean beam1Broken = !beam1.getState();
            boolean beam2Broken = !beam2.getState();

            // ---- Chassis ----
            robotCentricDrive(leftFront, leftBack, rightFront, rightBack,
                    gamepad2.left_trigger, gamepad2.right_trigger, rate);

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // ===========================================================
            // Turret aiming
            // 0.5 = behind the robot (half turn), 0.075/0.925 = front, ±180° travel.
            // ===========================================================
            robot_pose_x = follower.getPose().getX();
            robot_pose_y = follower.getPose().getY();

            robot_angle = follower.getPose().getHeading(); // 0 to 2π
            robot_velocity_magnitude = follower.getVelocity().getMagnitude();

            turret_field_x = robot_pose_x
                    + TURRET_OFFSET_X * Math.cos(robot_angle)
                    - TURRET_OFFSET_Y * Math.sin(robot_angle);
            turret_field_y = robot_pose_y
                    + TURRET_OFFSET_X * Math.sin(robot_angle)
                    + TURRET_OFFSET_Y * Math.cos(robot_angle);

            distanceToGoal = getDistanceToGoal(turret_field_x, turret_field_y,
                    goal_pose_x, goal_pose_y);

            robot_velocity_angle = Math.atan2(
                    follower.getVelocity().getYComponent(),
                    follower.getVelocity().getXComponent()
            );

            bearingToGoal = Math.atan2(
                    goal_pose_y - turret_field_y,
                    goal_pose_x - turret_field_x
            );
            bearingToGoal = (bearingToGoal + 2 * Math.PI) % (2 * Math.PI);

            // Reference aim to "behind the robot" by adding PI -> goal directly
            // behind == angleToTurn 0 == servo 0.5.
            double angleToTurn = normalizeAngle(bearingToGoal - robot_angle + Math.PI);
            degreesToTurn = Math.toDegrees(angleToTurn);

            double clampedDegrees = clamp(degreesToTurn, -TURRET_MAX_DEGREES, TURRET_MAX_DEGREES);
            turret_target_position = TURRET_CENTER + degreesToServoOffset(clampedDegrees);

            double alpha = computeLaunchAngle(distanceToGoal, goalHeight, theta);
            alpha = clamp(alpha, Math.toRadians(38), Math.toRadians(60));
            double v0 = computeLaunchSpeed(alpha, distanceToGoal, goalHeight);

            turret_target_position -= degreesToServoOffset(computeTurretOffset(
                    robot_velocity_magnitude, robot_velocity_angle, bearingToGoal, v0, alpha));

            // Fine-tune offset
            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                turretOffset -= 0.005;
            }
            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
                turretOffset += 0.005;
            }
            turret_target_position += turretOffset;

            // HARD safety clamp — never let the cable tangle
            turret_target_position = clamp(turret_target_position, TURRET_MIN, TURRET_MAX);

            turretServo1.setPosition(turret_target_position);
            turretServo2.setPosition(TURRET_MIRROR ? (1.0 - turret_target_position) : turret_target_position);
            // ===========================================================
            // End turret aiming
            // ===========================================================
            hoodTargetPos=hoodAngle(distanceToGoal, velocity);
            targetVelocity = 0; //flywheelSpeed(distanceToGoal);
            // ---- Intake + Transfer: right bumper = collect, left bumper = reverse ----
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                if (intakeController.currentStatus == IntakeController.IntakeStatus.COLLECT) {
                    intakeController.currentStatus   = OFF;
                    transferController.currentStatus = TransferController.TransferStatus.OFF;
                } else {
                    intakeController.currentStatus   = IntakeController.IntakeStatus.COLLECT;
                    transferController.currentStatus = TransferController.TransferStatus.COLLECT;
                }
            }

            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
                if (intakeController.currentStatus == IntakeController.IntakeStatus.REVERSE) {
                    intakeController.currentStatus   = OFF;
                    transferController.currentStatus = TransferController.TransferStatus.OFF;
                } else {
                    intakeController.currentStatus   = IntakeController.IntakeStatus.REVERSE;
                    transferController.currentStatus = TransferController.TransferStatus.REVERSE;
                }
            }

            // ---- Turret enable ----
            if (turretOn)
                turretController.currentStatus = TurretController.TurretStatus.RUNTO;
            else
                turretController.currentStatus = TurretController.TurretStatus.INIT;


            // ===========================================================
            // Shooting sequence (B button)
            // First press  -> stop feed, fire stopper, spin up flywheel.
            // After delay  -> feed balls into the spinning flywheel.
            // Second press -> end the shot, everything off.
            // ===========================================================
            // First press — start the shot
            if (currentGamepad2.b && !previousGamepad2.b
                    && !shooting
                    && shootEndTime.seconds() > shootEndCounter) {
                intakeController.currentStatus   = IntakeController.IntakeStatus.OFF;
                transferController.currentStatus = TransferController.TransferStatus.OFF;
                stopperController.currentStatus  = StopperController.StopperStatus.SHOOT;
                shootTime.reset();
                flywheelOn = true;
                shooting   = true;
            }

            // After spin-up delay — feed balls into the flywheel
            if (shooting && flywheelOn && shootTime.seconds() > delayShoot) {
                intakeController.currentStatus   = IntakeController.IntakeStatus.COLLECT;
                transferController.currentStatus = TransferController.TransferStatus.COLLECT;
                shootTransferTime.reset();
                shootTransfer = true;
                flywheelOn    = false;
            }

            // Second press — end the shot
            if (currentGamepad2.b && !previousGamepad2.b
                    && shootTransfer
                    && shootTransferTime.seconds() > 0.2) {
                stopperController.currentStatus  = StopperController.StopperStatus.NOSHOOT;
                intakeController.currentStatus   = OFF;
                transferController.currentStatus = TransferController.TransferStatus.OFF;
                flywheelOn    = false;
                shootTransfer = false;
                shooting      = false;
                shootEndTime.reset();
            }

            // Rumble while the stopper is firing
            if (stopperController.currentStatus == StopperController.StopperStatus.SHOOT) {
                gamepad2.rumble(1, 1, 100);
            }

            // ===========================================================
            // Beam-based auto-off safeties
            // Runs AFTER all input handling so the freshly-set status is
            // visible this same loop -> reverse can never be cancelled by a
            // one-frame race. While reversing, ALL of these are skipped.
            // ===========================================================
            boolean reversing =
                    intakeController.currentStatus   == IntakeController.IntakeStatus.REVERSE
                            && transferController.currentStatus == TransferController.TransferStatus.REVERSE;

            // treibile: true only when BOTH beams stay broken continuously for delayIntake seconds.
            // Forced false while reversing (and countdown reset so it can't latch the instant
            // reverse stops with both beams still broken).
            if (beam1Broken && beam2Broken && !reversing) {
                treibile = (bothBeamsTimer.seconds() > delayIntake);
            } else {
                bothBeamsTimer.reset();   // either beam cleared OR reversing -> restart the countdown
                treibile = false;
            }

            // ---- Stop intake when treibile is true (but NOT while shooting or reversing) ----
            if (treibile && !shooting && !reversing) {
                intakeController.currentStatus   = OFF;
                transferController.currentStatus = TransferController.TransferStatus.OFF;
                stopperController.currentStatus  = StopperController.StopperStatus.SHOOT;
            } else if (!shooting) {
                stopperController.currentStatus  = StopperController.StopperStatus.NOSHOOT;
            }

            // ---- Auto-off intake when beam1 stays broken for delayIntake seconds ----
            // Skipped while shooting (feeding) and while reversing (clearing a jam).
            if (beam1Broken && !shooting && !reversing) {
                if (beam1Timer.seconds() > delayIntake) {
                    intakeController.currentStatus   = OFF;
                    transferController.currentStatus = TransferController.TransferStatus.OFF;
                }
            } else {
                beam1Timer.reset();   // beam clear, shooting, OR reversing -> restart the countdown
            }
            // ===========================================================
            // End beam-based auto-off safeties
            // ===========================================================

            // ===========================================================
            // Flywheel PID
            // Runs continuously at SHOOT_VELOCITY (always on).
            // ===========================================================
            /*
            targetVelocity = SHOOT_VELOCITY;

            controller.setPIDF(P, I, kD, kV * targetVelocity + kS);
            velocity = -motor1.getVelocity();
            double power = controller.calculate(targetVelocity - velocity);
            motor.setPower(power);
            motor1.setPower(power);*/

            // ===========================================================
            // Transfer jam protection
            // If transfer current > limit AND transfer is COLLECT AND intake
            // is COLLECT, turn ONLY the transfer OFF (intake keeps running).
            // Skipped while shooting, since feeding draws high current on purpose.
            // ===========================================================
            if (!shooting
                    && robot.transfer.getCurrent(CurrentUnit.AMPS) > TRANSFER_AMP_LIMIT
                    && transferController.currentStatus == TransferController.TransferStatus.COLLECT
                    && intakeController.currentStatus   == IntakeController.IntakeStatus.COLLECT) {
                transferController.currentStatus = TransferController.TransferStatus.OFF;
            }
            // ===========================================================
            // End transfer jam protection
            // ===========================================================

            // ---- Updates ----
            intakeController.update();
            transferController.update();
            stopperController.update();
            hoodController.update(hoodTargetPos);
            //turretController.update(turret_target_position);

            // ---- Telemetry ----
            dashTelemetry.addData("Beam 1",             beam1Broken ? "BROKEN (object)" : "clear");
            dashTelemetry.addData("Beam 2",             beam2Broken ? "BROKEN (object)" : "clear");
            dashTelemetry.addData("treibile",           treibile);
            dashTelemetry.addData("reversing",          reversing);
            dashTelemetry.addData("bothBeamsTimer",     bothBeamsTimer.seconds());
            dashTelemetry.addData("amperaj intake",     intakeController.intake.getCurrent(CurrentUnit.AMPS));
            dashTelemetry.addData("amperaj transfer",   robot.transfer.getCurrent(CurrentUnit.AMPS));
            dashTelemetry.addData("intakeStatus",       intakeController.currentStatus);
            dashTelemetry.addData("transferStatus",     transferController.currentStatus);
            dashTelemetry.addData("stopperStatus",      stopperController.currentStatus);
            dashTelemetry.addData("shooting",           shooting);
            dashTelemetry.addData("targetVel",          targetVelocity);
            dashTelemetry.addData("curVel",             velocity);
            dashTelemetry.addData("distanceToGoal",     distanceToGoal);
            dashTelemetry.addData("turret_field_x",     turret_field_x);
            dashTelemetry.addData("turret_field_y",     turret_field_y);
            dashTelemetry.addData("degreesToTurn",      degreesToTurn);
            dashTelemetry.addData("turret_target_pos",  turret_target_position);
            dashTelemetry.addData("turretOffset",       turretOffset);
            dashTelemetry.addData("robot_angle_deg",    Math.toDegrees(robot_angle));
            dashTelemetry.addData("position",           follower.getPose());
            dashTelemetry.update();
            telemetry.update();
        }
    }

    // ============================================================
    // 1. Launch angle from distance, height, and entry angle
//    x = horizontal distance to goal (from your distanceToGoal)
//    y = height of goal above your launch point (constant, measure it)
//    theta = desired entry angle in radians (constant, you pick it, negative because ball comes down)
    private static double computeLaunchAngle(double x, double y, double theta) {
        return Math.atan(2 * y / x - Math.tan(theta));
    }

    // 2. Launch speed from launch angle, distance, and height
    private static double computeLaunchSpeed(double alpha, double x, double y) {
        double g = 386.1; // gravity in in/s², use 9.8 if you work in meters
        return Math.sqrt(
                (g * x * x) / (2 * Math.cos(alpha) * Math.cos(alpha) * (x * Math.tan(alpha) - y))
        );
    }

    // 3. Hood servo position from launch angle (linear interpolation)
//    a1, a2 = your min and max hood angles in radians (measured)
//    s1, s2 = corresponding servo positions (measured)
    private static double computeServoPosition(double alpha, double a1, double s1, double a2, double s2) {
        return ((s1 - s2) / (a1 - a2)) * (alpha - a1) + s1;
    }

    // 4. Flywheel RPM from launch speed (your calibration line of best fit)
//    m = slope, b = intercept (from your empirical data)
    private static double computeFlywheelTPS(double v0) {
        return 8.60347 * v0 - 367.05262;
    }

// --- Velocity compensation (section B) ---

    // 5. Decompose robot velocity into radial and tangential relative to goal
//    robotVelMag = magnitude of robot velocity
//    robotVelAngle = angle of robot velocity vector
//    angleToGoal = bearing from robot to goal (you already compute this)
    private static double[] decomposeVelocity(double robotVelMag, double robotVelAngle, double angleToGoal) {
        double deltaTheta = robotVelAngle - angleToGoal;
        double radial = -Math.cos(deltaTheta) * robotVelMag;     // toward/away from goal
        double tangential = Math.sin(deltaTheta) * robotVelMag;  // perpendicular to goal
        return new double[]{radial, tangential};
    }

    // 6. Compensated horizontal velocity
//    vx = original horizontal launch velocity = v0 * cos(alpha)
//    t = time of flight = x / (v0 * cos(alpha))
//    radial, tangential from step 5
    private static double computeVxNew(double v0, double alpha, double radial, double tangential) {
        double vxCompensated = v0 * Math.cos(alpha) + radial;
        return Math.sqrt(vxCompensated * vxCompensated + tangential * tangential);
    }

    // 7. New launch angle after velocity compensation
    private static double computeCompensatedAngle(double alpha, double v0, double vxNew) {
        return Math.atan(v0 * Math.sin(alpha) / vxNew);
    }

    // 8. Turret offset angle to account for tangential movement
    public static double computeTurretOffset(double robot_velocity_magnitude, double robot_velocity_angle, double bearingToGoal, double v0, double alpha) {
        double deltaTheta = robot_velocity_angle - bearingToGoal;
        double radial = -Math.cos(deltaTheta) * robot_velocity_magnitude;
        double tangential = Math.sin(deltaTheta) * robot_velocity_magnitude;
        double vxCompensated = v0 * Math.cos(alpha) + radial;
        double offsetRadians = Math.atan(tangential / vxCompensated);
        return Math.toDegrees(offsetRadians) / 360.0;
    }

    // ============================================================
    // Geometry helpers
    // ============================================================
    private double normalizeAngle(double angle) {
        while (angle >  Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public double getDistanceToGoal(double robotX, double robotY,
                                    double goalX,  double goalY) {
        double dx = goalX - robotX;
        double dy = goalY - robotY;
        return Math.sqrt(dx * dx + dy * dy);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    // Maps a turn in degrees (±180) to a servo-unit offset from center.
    // ±180° -> ±0.425 servo units, so 0.5±0.425 = [0.075, 0.925].
    private static double degreesToServoOffset(double degrees) {
        return (degrees / TURRET_MAX_DEGREES) * TURRET_HALF_RANGE;
    }

    public static double hoodAngle(double distanceToGoal, double actualVelocity) {
        double alpha = computeLaunchAngle(distanceToGoal, goalHeight, theta);
        alpha = clamp(alpha, Math.toRadians(30), Math.toRadians(50));
        double actualV0 = (actualVelocity + 367.05262) / 8.60347;
        double v0 = computeLaunchSpeed(alpha, distanceToGoal, goalHeight);
        double speedRatio = actualV0 / v0;
        if(actualVelocity < 200){
            speedRatio = 1;
        }
        double[] decomposedVelocity = decomposeVelocity(robot_velocity_magnitude, robot_velocity_angle, bearingToGoal);
        double vxNew=computeVxNew(v0, alpha, decomposedVelocity[0], decomposedVelocity[1]);
        double alphaNew = computeCompensatedAngle(alpha, v0, vxNew) / speedRatio;
        double safeAlpha = clamp(alphaNew, Math.toRadians(30), Math.toRadians(50));
        return computeServoPosition(safeAlpha, Math.toRadians(30), 0.37, Math.toRadians(50), 0.8);
    }

    public static double flywheelSpeed(double distanceToGoal) {
        double alpha = computeLaunchAngle(distanceToGoal, goalHeight, theta);
        alpha = clamp(alpha, Math.toRadians(30), Math.toRadians(50));
        double v0 = computeLaunchSpeed(alpha, distanceToGoal, goalHeight);
        double[] decomposedVelocity = decomposeVelocity(robot_velocity_magnitude, robot_velocity_angle, bearingToGoal);
        double vx=computeVxNew(v0, alpha, decomposedVelocity[0], decomposedVelocity[1]);
        double vy=v0 * Math.sin(alpha);
        double v0New=Math.sqrt(vx*vx + vy*vy);
        double tps = computeFlywheelTPS(v0New);
        return clamp(tps, 1000, 2200);
    }
}