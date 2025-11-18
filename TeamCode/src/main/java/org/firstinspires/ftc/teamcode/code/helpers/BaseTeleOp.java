package org.firstinspires.ftc.teamcode.code.helpers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

@Config
public abstract class BaseTeleOp extends OpMode {
    protected Limelight3A limelight;

    // TODO: Check if these are correct
    public static final double yOffset = -156.0; // -168.0 // mm
    public static final double xOffset = -72.0; // -84.0 // mm

    protected Pose2D[] presetPositions;

    public int currentPreset = -1;

    public double velocity = 2800;

    protected OdometryHolonomicDrivetrain driveTrain;
    protected AutoAligner autoAligner;

    protected boolean autoLock = false;

    protected Intake intake;
    protected boolean intakeOn = false;
    protected boolean intakeReversed = false;

    protected Turret turret;
    protected double shooterVelocity;
    protected double hoodAngle;
    protected double turretAngle;

    protected abstract boolean isRedAlliance();
    protected abstract Pose2D[] getPresetPositions();

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

        driveTrain.setCountsToSlowDown(500);

        // Initialize AutoAligner
        autoAligner = new AutoAligner(driveTrain, limelight, isRedAlliance());

        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intake"));
        turret = new Turret(
                hardwareMap.get(DcMotorEx.class, "shooter"),
                hardwareMap.get(DcMotorEx.class, "turret"),
                hardwareMap.get(Servo.class, "hood"),
                hardwareMap.get(Servo.class, "blocker")
        );

        presetPositions = getPresetPositions();
    }

    @Override
    public void loop() {
        driveTrain.updatePosition();
        Pose2D currentPos = driveTrain.getPosition();

        telemetry.addData("Position", "X: %.2f, Y: %.2f, H: %.2f",
                currentPos.getX(DistanceUnit.INCH),
                currentPos.getY(DistanceUnit.INCH),
                currentPos.getHeading(AngleUnit.DEGREES));

        // Use AutoAligner to get wanted heading and distance
        double wantedHeading = autoAligner.getAutoAlignAngle();

        if (currentPreset >= 0 && (Math.abs(gamepad1.left_stick_x) > 0.001 || Math.abs(gamepad1.left_stick_y) > 0.001 || Math.abs(gamepad1.right_stick_x) > 0.001)) {
            currentPreset = -1;
            autoLock = true;
        }

        if (currentPreset >= 0) {
            Pose2D wantedPosition = new Pose2D(
                DistanceUnit.INCH,
                presetPositions[currentPreset].getX(DistanceUnit.INCH),
                presetPositions[currentPreset].getY(DistanceUnit.INCH),
                AngleUnit.DEGREES,
                wantedHeading
            );
            driveTrain.setPositionDrive(wantedPosition, velocity);
        } else {
            double turn = -gamepad1.right_stick_x * velocity;

            if (gamepad1.y) {
                autoLock = true;
            }

            if (Math.abs(turn) > 2) {
                autoLock = false;
            }

            if (autoLock) {
                driveTrain.setWantedHeading(wantedHeading);
                turn = driveTrain.getHeadingCorrectionVelocity();
            }
            driveTrain.setVelocityDriveFieldCentric(-gamepad1.left_stick_y * velocity, -gamepad1.left_stick_x * velocity, turn, isRedAlliance() ? -90 : 90);
        }

        if (autoLock) {
            // Use AutoAligner methods to calculate shooter angle and velocity
            hoodAngle = autoAligner.getOptimalHoodAngle();
            shooterVelocity = autoAligner.getShooterVelocityFromAngle(hoodAngle);
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
            intakeOn = !intakeOn;
        }
        if (gamepad1.bWasPressed()) {
            intakeReversed = !intakeReversed;
        }

        if (gamepad1.dpadRightWasPressed()) {
            shooterVelocity = 0;
        }

        double intakeVelocity = intakeOn ? (intakeReversed ? -2000 : 2000) : 0;

        if (gamepad1.a) {
            turret.open();
            intakeVelocity = 0;

            if (Math.abs(turret.getVelocity() - shooterVelocity) <= 60) {
                intakeVelocity = 2000;
            }
        } else {
            turret.close();
        }

        // Preset & Auto Lock
        if (gamepad1.rightBumperWasPressed()) {
            currentPreset = 0;
        } else if (gamepad1.leftBumperWasPressed()) {
            currentPreset = 1;
        }

        // Gamepad 2 controls
        // Right Bumper: Turn up shooter velocity by 100
        // Left Bumper: Turn down shooter velocity by 100
        // D-Pad Up: Increase hood angle by 2 degrees (More direct, 90 is straight forward)
        // D-Pad Down: Decrease hood angle by 2 degrees (More parabolic, 0 is straight up)
        if (gamepad2.rightBumperWasPressed()) {
            shooterVelocity = Math.round(shooterVelocity / 100) * 100;
            shooterVelocity += 100;
        }
        if (gamepad2.leftBumperWasPressed()){
            shooterVelocity = Math.round(shooterVelocity / 100) * 100;
            shooterVelocity -= 100;
        }

        if (gamepad2.dpadUpWasPressed()) {
            hoodAngle += 2;
        }
        if (gamepad2.dpadDownWasPressed()) {
            hoodAngle -= 2;
        }
        if (gamepad2.dpad_left) {
            turretAngle += 5;
        }
        if (gamepad2.dpad_right) {
            turretAngle -= 5;
        }

        if (gamepad2.aWasPressed()) {
            shooterVelocity = 2000;
        }
        if (gamepad2.xWasPressed()) {
            shooterVelocity = 0;
        }

        hoodAngle = Range.clip(hoodAngle, 0, 50);
        turretAngle = Range.clip(turretAngle, -90, 90);
        shooterVelocity = Range.clip(shooterVelocity, 0, 2800);

        intake.setVelocity(intakeVelocity);
        turret.setVelocity(shooterVelocity);
        turret.setHoodAngle(hoodAngle);
        turret.setTurretAngle(turretAngle);

        driveTrain.drive();

//        Log.d("PID", "Current Velocity: " + shooter.getVelocity());
//        Log.d("PID", "Wanted Velocity: " + shooterVelocity);
//        telemetry.addData("Current Velocity", shooter.getVelocity());
//        telemetry.addData("Wanted Velocity", shooterVelocity);
//        telemetry.update();
//        telemetry.addData("Turret Given Target Angle", turretAngle);
//        telemetry.addData("Turret Current Angle", turret.getCurrentAngle());
//        telemetry.addData("Turret True Target Angle", turret.getTargetAngle());
//        telemetry.update();
    }
}