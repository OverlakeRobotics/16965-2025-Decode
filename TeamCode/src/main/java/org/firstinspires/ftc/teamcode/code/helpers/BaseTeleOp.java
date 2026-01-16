package org.firstinspires.ftc.teamcode.code.helpers;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.code.parts.Intake;
import org.firstinspires.ftc.teamcode.code.parts.Turret;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.PathServer;

@Config
public abstract class BaseTeleOp extends OpMode {
    protected Limelight3A limelight;

    // TODO: Check if these are correct
    public static final double yOffset = -156.0; // -168.0 // mm
    public static final double xOffset = 72.0; // -84.0 // mm
    public static double shooterTolerance = 80;
    public static double turretTolerance = 5;

    public static double hoodAngleVelScale = 0.5;

    public static double ADJUSTMENT_FACTOR = 0.021; //0.021;

    protected static final class Preset {
        public final Pose2D[] poses;
        public final boolean useAutoAlignHeading;

        public Preset(Pose2D pose, boolean useAutoAlignHeading) {
            this(new Pose2D[]{pose}, useAutoAlignHeading);
        }

        public Preset(Pose2D[] poses, boolean useAutoAlignHeading) {
            this.poses = poses;
            this.useAutoAlignHeading = useAutoAlignHeading;
        }
    }

    protected Preset[] presetPositions;

    public int currentPreset = -1;

    public double velocity = 2800;

    protected OdometryHolonomicDrivetrain driveTrain;
    protected AutoAligner autoAligner;

    protected boolean autoLock = false;
    protected boolean autoShooter = true;

    protected Intake intake;
    protected boolean intakeOn = false;
    protected boolean intakeReversed = false;

    protected Turret turret;
    protected double shooterVelocity;
    protected double hoodAngle;
    protected double turretAngle;
    protected boolean autoTurret = true;

    protected abstract boolean isRedAlliance();
    protected Preset[] getPresetPositions() {
        double ySign = isRedAlliance() ? -1.0 : 1.0;
        return new Preset[]{
                new Preset(new Pose2D(DistanceUnit.INCH, -54, 15 * ySign, AngleUnit.DEGREES, 0), true),
                new Preset(new Pose2D(DistanceUnit.INCH, 15, 15 * ySign, AngleUnit.DEGREES, 0), true),
                new Preset(new Pose2D[]{new Pose2D(DistanceUnit.INCH, -12, 38 * ySign, AngleUnit.DEGREES, 90 * ySign),
                                        new Pose2D(DistanceUnit.INCH, -12, 60 * ySign, AngleUnit.DEGREES, 60 * ySign)},
                        false),
        };
    }

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();

        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpointDriver.setOffsets(xOffset, yOffset, DistanceUnit.MM);
        driveTrain = new OdometryHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                new GoBildaPinpointOdometry(pinpointDriver)
        );

        turret = new Turret(
                hardwareMap.get(DcMotorEx.class, "shooterTop"),
                hardwareMap.get(DcMotorEx.class, "shooterBottom"),
                hardwareMap.get(DcMotorEx.class, "turret"),
                hardwareMap.get(Servo.class, "hood"),
                hardwareMap.get(Servo.class, "blocker"),
                hardwareMap.get(AnalogInput.class, "potentiometer")
        );
//        turret.resetTurretEncoder();
//        turret.setEncoderOffset();
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intake"));

        // Initialize AutoAligner
        autoAligner = new AutoAligner(driveTrain, turret, limelight, isRedAlliance());

        presetPositions = getPresetPositions();
        PathServer.startServer();
    }

    @Override
    public void init_loop() {
        PathServer.setRobotPose(driveTrain.getPosition());
    }

    @Override
    public void loop() {
        driveTrain.updatePosition();
        Pose2D currentPos = driveTrain.getPosition();
        PathServer.setRobotPose(currentPos);
        autoAligner.updateInterpolation(currentPos.getX(DistanceUnit.INCH), currentPos.getY(DistanceUnit.INCH));

        Log.d("Turret Error", "Turret Error: " + (turret.getTurretTargetAngle() - turret.getTurretCurrentAngle()));

        telemetry.addData("Position", "X: %.2f, Y: %.2f, H: %.2f",
                currentPos.getX(DistanceUnit.INCH),
                currentPos.getY(DistanceUnit.INCH),
                currentPos.getHeading(AngleUnit.DEGREES));

        // Use AutoAligner to get wanted heading and distance
        double wantedHeading = autoAligner.getDrivetrainAutoAlignAngle();

        if (currentPreset >= 0 && (Math.abs(gamepad1.left_stick_x) > 0.001 || Math.abs(gamepad1.left_stick_y) > 0.001 || Math.abs(gamepad1.right_stick_x) > 0.001)) {
            currentPreset = -1;
            autoLock = true;
        }

        double turretAdjustment = 0;

        if (currentPreset >= 0) {
            Preset preset = presetPositions[currentPreset];
            driveTrain.setVelocity((int) velocity);
            driveTrain.setPositionDrive(preset.poses, driveTrain.getNextPointIndex());
            if (preset.useAutoAlignHeading) {
                Pose2D nextPoint = preset.poses[driveTrain.getNextPointIndex()];
                preset.poses[driveTrain.getNextPointIndex()] = new Pose2D(DistanceUnit.INCH,
                        nextPoint.getX(DistanceUnit.INCH), nextPoint.getY(DistanceUnit.INCH),
                        AngleUnit.DEGREES, autoAligner.getDrivetrainAutoAlignAngle());
            }
        } else {
            double turn = -gamepad1.right_stick_x * velocity;

            if (gamepad1.y) {
                autoLock = true;
                autoShooter = true;
            }

            if (Math.abs(turn) > 2) {
                autoLock = false;
            }

            if (autoLock) {
                driveTrain.setWantedHeading(wantedHeading);
                turn = driveTrain.getHeadingCorrectionVelocity();
            }
            driveTrain.setVelocityDriveFieldCentric(-gamepad1.left_stick_y * velocity, -gamepad1.left_stick_x * velocity, turn, isRedAlliance() ? -90 : 90);

            turretAdjustment = -turn * ADJUSTMENT_FACTOR;
        }

        if (autoShooter) {
            // Use AutoAligner methods to calculate shooter angle and velocity
            hoodAngle = autoAligner.getOptimalHoodAngle() - autoAligner.lastVelPar * hoodAngleVelScale;
            hoodAngle = Range.clip(hoodAngle, turret.minHoodAngle, turret.maxHoodAngle);
            shooterVelocity = autoAligner.getShooterVelocityFromAngle(hoodAngle);
        }
//        turretAngle = autoAligner.getTurretAutoAlignAngle() + turretAdjustment;

        if (autoTurret) {
            turretAngle = autoAligner.getTurretAutoAlignAngle() + turretAdjustment;
        }

        // Gamepad 1 controls
        // Right Bumper: Far preset
        // Left Bumper: Close preset
        // A: Shoot artifacts (hold down)
        // X: Turn on/off intake
        // B: Reverse intake direction
        // Y: Auto aim
        // D-Pad Right: Turn off shooter

        if (gamepad1.xWasPressed()) {
            if (!intakeReversed) {
                intakeOn = !intakeOn;
            }
            intakeReversed = false;
        }
        if (gamepad1.bWasPressed()) {
            intakeReversed = true;
            intakeOn = true;
        }

        if (gamepad1.dpadRightWasPressed()) {
            autoShooter = false;
            shooterVelocity = 0;
        }
//        if (gamepad1.dpadLeftWasPressed()) {
//            autoTurret = !autoTurret;
//        }

        double intakeVelocity = intakeOn ? (intakeReversed ? -2800 : 2800) : 0;

        if (gamepad1.a) {
            turret.open();
            intakeVelocity = 0;

            if (Math.abs(turret.getShooterVelocity() - shooterVelocity) <= shooterTolerance && (Math.abs(turret.getTurretTargetAngle() - turret.getTurretCurrentAngle())) <= turretTolerance) {
                intakeVelocity = 2800;
            }
        } else {
            turret.close();
        }

        // Preset for gate pickup
        if (gamepad1.rightBumperWasPressed()) {
            currentPreset = 2;
        }

        // Gamepad 2 controls (only for backup in case of robot reset/pinpoint issues)

        // LAUNCHER BACKUP
        // Following four only work when autoShooter is off
        // Right Bumper: Turn up shooter velocity by 100
        // Left Bumper: Turn down shooter velocity by 100
        // D-Pad Up: Increase hood angle by 2 degrees (More direct, 90 is straight forward)
        // D-Pad Down: Decrease hood angle by 2 degrees (More parabolic, 0 is straight up)

        // TURRET BACKUP
        // A: Toggle autoTurret
        // Following three only work when autoTurret is off
        // D-Pad Right: Increase turret angle by 5 degrees (CCW)
        // D-Pad Left: Decrease turret angle by 5 degrees (CW)
        // Y: Reset turret encoder to 0

        // PINPOINT BACKUP
        // B: Reset pinpoint position based on limelight
        // X: Reset pinpoint heading to 0

        if (!autoShooter) {
            if (gamepad2.rightBumperWasPressed()) {
                shooterVelocity = Math.round(shooterVelocity / 100) * 100;
                shooterVelocity += 100;
            }
            if (gamepad2.leftBumperWasPressed()) {
                shooterVelocity = Math.round(shooterVelocity / 100) * 100;
                shooterVelocity -= 100;
            }
            if (gamepad2.dpadUpWasPressed()) {
                hoodAngle += 2;
            }
            if (gamepad2.dpadDownWasPressed()) {
                hoodAngle -= 2;
            }
        }

        if (gamepad2.aWasPressed()) {
            autoTurret = !autoTurret;
        }

        if (!autoTurret) {
            // Will need to check values for speed/precision
            if (gamepad2.dpad_left) {
                turretAngle += 5;
            }
            if (gamepad2.dpad_right) {
                turretAngle -= 5;
            }

            if (gamepad2.yWasPressed()) {
                turret.resetTurretEncoder();
                turretAngle = 0;
            }
        }

        if (gamepad2.bWasPressed()) {
//            autoAligner.resetPinpointPositionFromLimelight();
            double ySign = isRedAlliance() ? 1.0 : -1.0;
            Pose2D currPos = driveTrain.getPosition();
            driveTrain.setPosition(new Pose2D(DistanceUnit.INCH, -63, 63 * ySign, AngleUnit.DEGREES, currPos.getHeading(AngleUnit.DEGREES)));
        }
        if (gamepad2.xWasPressed()) {
            Pose2D pos = driveTrain.getPosition();
            double x = pos.getX(DistanceUnit.INCH);
            double y = pos.getY(DistanceUnit.INCH);
            driveTrain.setPosition(
                    new Pose2D(
                            DistanceUnit.INCH,
                            x,
                            y,
                            AngleUnit.DEGREES,
                            0
                    )
            );
        }



//        if (gamepad2.aWasPressed()) {
//            shooterVelocity = 2000;
//        }
//        if (gamepad2.xWasPressed()) {
//            shooterVelocity = 0;
//        }

        hoodAngle = Range.clip(hoodAngle, turret.minHoodAngle, turret.maxHoodAngle);
        turretAngle = Range.clip(turretAngle, turret.MIN_ANGLE_LIMIT, turret.MAX_ANGLE_LIMIT);
        shooterVelocity = Range.clip(shooterVelocity, 0, turret.MAX_VELOCITY);

        intake.setVelocity(intakeVelocity);
        turret.setShooterVelocity(shooterVelocity);
        turret.setHoodAngle(hoodAngle);
        turret.setTurretAngle(turretAngle);

        driveTrain.drive();

//        Log.d("PID", "Current Velocity: " + turret.getShooterVelocity());
//        Log.d("PID", "Wanted Velocity: " + shooterVelocity);
//        telemetry.addData("Current Velocity", turret.getShooterVelocity());
//        telemetry.addData("Wanted Velocity", shooterVelocity);
//        telemetry.addData("Turret Given Target Angle", turretAngle);
//        telemetry.addData("Turret Current Angle", turret.getTurretCurrentAngle());
//        telemetry.addData("Turret True Target Angle", turret.getTurretTargetAngle());
        telemetry.update();
    }

    @Override
    public void stop() {
        PathServer.stopServer();
    }
}
