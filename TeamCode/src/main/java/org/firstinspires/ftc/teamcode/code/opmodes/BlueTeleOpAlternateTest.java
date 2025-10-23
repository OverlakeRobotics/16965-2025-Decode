package org.firstinspires.ftc.teamcode.code.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.code.parts.Intake;
import org.firstinspires.ftc.teamcode.code.parts.Shooter;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;

import java.util.List;

@Config
@TeleOp(name = "Blue TeleOp Alternate Test", group = "TeleOp")
public class BlueTeleOpAlternateTest extends OpMode {
    public static final double g = 386.08858; // in/s^2
    public static final double rhinoWheelRadius = 1.88976; // in
    // TODO: Tune this value
    public static final double k_slip = 0.90; // estimated slip factor
    public static final double motorTicksPerRev = 28 * 4d;
    private Limelight3A limelight;

    public static final double yOffset = -168.0; // mm
    public static final double xOffset = -84.0; // mm

    public static final double goalX = 60;
    public static final double goalY = 54;
    // TODO: Measure actual vertical distance from launcher to goal entrance
    public static final double goalDZ = 20;

    // Positive angle is to the left, positive x is forward, and positive y is left
    // This is the center of the bot when the program is initialized
    public static final Pose2D startPos = new Pose2D(DistanceUnit.INCH, -63, 15, AngleUnit.DEGREES, 0);

    public static final Pose2D[] presetPositions = {
            new Pose2D(DistanceUnit.INCH, -54, 0, AngleUnit.DEGREES, 0),
            new Pose2D(DistanceUnit.INCH, 27, 21, AngleUnit.DEGREES, 0),
    };

    public static final int targetID = 20;

    public int currentPreset = -1;

    public double velocity = 2000;

    private OdometryHolonomicDrivetrain driveTrain;


    private boolean autoLock = false;

    private Intake intake;
    private boolean intakeOn = false;
    private boolean intakeReversed = false;

    private Shooter shooter;
    private double shooterVelocity;
    private double shooterAngle;

    // TODO: Check angle and velocity calculations
    // Distances should be passed in as inches due to FTC standard units
    // theta is the launch angle in degrees, where launching straight up is 0 degrees (hood flat) and straight forward is 90 degrees (hood vertical)
    // Returns -1 if no valid solution (i.e. not possible given the angle)
    public double getShooterVelocity(double horizontalDist, double verticalDist, double theta) {
        // Convert theta to radians
        theta = Math.toRadians(theta);
        // Impossible
        if (verticalDist >= horizontalDist / Math.tan(theta) || theta <= 0 || horizontalDist <= 0) {
            return -1;
        }
        // First calculate the required launch velocity (derived from kinematics assuming no air resistance)
        double v = Math.sqrt(
                g * Math.pow(horizontalDist, 2) /
                        (2 * Math.pow(Math.sin(theta), 2) * (horizontalDist / Math.tan(theta) - verticalDist))
        );
        // Then calculate RPM from linear velocity
        double rpm = (v * 60) / (2 * Math.PI * rhinoWheelRadius * k_slip);
        // Scale to motor ticks per second
        // TODO: Check ticks/sec calculation
        return rpm * motorTicksPerRev / 60;
    }

    public double getBestHoodAngleDegrees(double horizontalDist, double verticalDist) {
        // Guard bad input
        if (!(horizontalDist > 0)) return Double.NaN;

        final double minTheta = 8.0;    // base angle when very close
        final double k = 28.0;  // overall rise (increase for steeper far shots)
        final double midpointDist = 36.0;  // midpoint distance (in)
        final double rampFactor = 24.0;  // how quickly it ramps (in)

        final double minHoodDeg = 0.5;     // avoid exact 0°
        final double maxHoodDeg = 45.0;    // your mechanical cap
        final double epsPhysDeg = 0.5;     // safety margin vs physics cap

        // 1) Base angle from smooth monotone map
        double thetaBase = minTheta + k * Math.toDegrees(Math.atan((horizontalDist - midpointDist) / rampFactor));

        // 2) Physics cap
        // Feasible iff horizontalDist / tan(theta) > verticalDist.
        // For Δh>0, that implies theta < atan(D/Δh).
        double thetaPhysCapDeg = Math.toDegrees(Math.atan(horizontalDist / verticalDist)) - epsPhysDeg;
        if (thetaPhysCapDeg < minHoodDeg) return Double.NaN; // no feasible angle in 0–45°

        // 3) Apply mech + physics caps
        double theta = Range.clip(thetaBase, minHoodDeg, maxHoodDeg);
        if (theta > thetaPhysCapDeg) {
            theta = Math.max(minHoodDeg, Math.min(maxHoodDeg, thetaPhysCapDeg));
        }

        // 4) Final feasibility sanity check (numerical guard)
        double th = Math.toRadians(theta);
        double denomGeom = horizontalDist / Math.tan(th) - verticalDist; // must be > 0
        if (denomGeom <= 1e-6) return Double.NaN;

        return theta;
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
        driveTrain.setPosition(startPos);
        driveTrain.setCountsToSlowDown(500);

        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intakeMotor"));
        shooter = new Shooter(
                hardwareMap.get(DcMotorEx.class, "shooter"),
                hardwareMap.get(Servo.class, "hood"),
                hardwareMap.get(Servo.class, "blocker")
        );
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        LLResultTypes.FiducialResult targetApril = null;
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();
            for (LLResultTypes.FiducialResult tag : aprilTags) {
                telemetry.addData("April Tag", "ID: %d, Family: %s, X: %.2f, Y: %.2f", tag.getFiducialId(), tag.getFamily(), tag.getTargetXDegrees(), tag.getTargetYDegrees());
                if (tag.getFiducialId() == targetID) {
                    targetApril = tag;
                    break;
                }
            }
        }

        driveTrain.updatePosition();
        Pose2D currentPos = driveTrain.getPosition();

        telemetry.addData("Position", "X: %.2f, Y: %.2f, H: %.2f", currentPos.getX(DistanceUnit.INCH), currentPos.getY(DistanceUnit.INCH), currentPos.getHeading(AngleUnit.DEGREES));

        double wantedHeading;
        // TODO: Check if distance needs to be increased by a factor so it's to the inside of the goal
        double distance;
        if (targetApril != null) {
            wantedHeading = driveTrain.getPosition().getHeading(AngleUnit.DEGREES) - targetApril.getTargetXDegrees();
            Position relativePose = targetApril.getRobotPoseTargetSpace().getPosition();
            distance = Math.hypot(relativePose.x, relativePose.y);
        } else {
            Pose2D pos = driveTrain.getPosition();
            double dy = goalY - pos.getY(DistanceUnit.INCH);
            double dx = goalX - pos.getX(DistanceUnit.INCH);
            wantedHeading = Math.toDegrees(Math.atan2(dy, dx));
            distance = Math.hypot(dx, dy);
        }

        // Preset & Auto Lock
        if (gamepad2.a) {
            currentPreset = 0;
        } else if (gamepad2.b) {
            currentPreset = 1;
        }

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

            driveTrain.setVelocityDriveFieldCentric(-gamepad1.left_stick_y * velocity, -gamepad1.left_stick_x * velocity, turn);
        }

        if (autoLock) {
            shooterAngle = getBestHoodAngleDegrees(distance, goalDZ);
            shooterVelocity = getShooterVelocity(distance, goalDZ, shooterAngle);
        }

        // Shooter and Intake
        if (gamepad1.rightBumperWasPressed()) {
            shooterVelocity = Math.round(shooterVelocity / 100) * 100;
            shooterVelocity += 100;
        }
        if (gamepad1.leftBumperWasPressed()){
            shooterVelocity = Math.round(shooterVelocity / 100) * 100;
            shooterVelocity -= 100;
        }

        if (gamepad1.dpadUpWasPressed()) {
            shooterAngle += 2;
        }
        if (gamepad1.dpadDownWasPressed()) {
            shooterAngle -= 2;
        }

        if (gamepad1.xWasPressed()) {
            intakeOn = !intakeOn;
        }
        if (gamepad1.bWasPressed()) {
            intakeReversed = !intakeReversed;
        }

        double intakeVelocity = intakeOn ? (intakeReversed ? -2000 : 2000) : 0;

        if (gamepad1.a) {
            shooter.open();
            intakeVelocity = 0;

            if (Math.abs(1 - (shooter.getVelocity() / shooterVelocity)) < 0.03) {
                intakeVelocity = 1000;
            }
        } else {
            shooter.close();
        }

        shooterAngle = Range.clip(shooterAngle, 0, 60);
        shooterVelocity = Range.clip(shooterVelocity, 0, 2000);

        intake.setVelocity(intakeVelocity);
        shooter.setVelocity(shooterVelocity);
        shooter.setVelocity(shooterAngle);

        telemetry.addData("Shooter Angle", shooterAngle);
        telemetry.addData("Shooter Velocity", shooterVelocity);

        telemetry.update();
        driveTrain.drive();
    }
}
