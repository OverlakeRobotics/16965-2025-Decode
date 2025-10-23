package org.firstinspires.ftc.teamcode.code.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.code.parts.Intake;
import org.firstinspires.ftc.teamcode.code.parts.Shooter;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;

import java.util.List;

@Config
@TeleOp(name = "Blue TeleOp", group = "TeleOp")
public class BlueTeleOp extends OpMode {
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

    // TODO: Find actual constant to multiply by (currently 0.9), also check that the ticks per revolution of 112 is correct
    private static final double ticksToLaunchVelocity = (2 * Math.PI / 28) * (48.0 / 254.0) * 0.9; // In inches/s

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

    // TODO: Check if this works
    // All inches parameters
    public static double solveLaunchAngle(double x, double v) {
        double g = 0.249174; // Gravity in inches/s^2

        double v2 = v * v;
        double gTerm = g * x * x / (2 * v2);
        double discriminant = x * x - 4 * gTerm * (gTerm + goalDZ);

        if (discriminant < 0) {
            return Double.NaN;
        }

        double tanTheta1 = (x - Math.sqrt(discriminant)) / (2 * gTerm);

        return Math.toDegrees(Math.atan(tanTheta1));
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
            shooterVelocity = Math.min(1000 + (distance / 144) * 800, 1800);
            shooterAngle = solveLaunchAngle(distance, shooterVelocity * ticksToLaunchVelocity);
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

            if (Math.abs(shooter.getVelocity() - shooterVelocity) <= 40) {
                intakeVelocity = 1000;
            }
        } else {
            shooter.close();
        }

        shooterAngle = Range.clip(shooterAngle, 0, 60);
        shooterVelocity = Range.clip(shooterVelocity, 0, 2000);

        intake.setVelocity(intakeVelocity);
        shooter.setVelocity(shooterVelocity);
        shooter.setAngle(shooterAngle);

        telemetry.addData("Shooter Angle", shooterAngle);
        telemetry.addData("Shooter Velocity", shooterVelocity);

        telemetry.update();
        driveTrain.drive();
    }
}
