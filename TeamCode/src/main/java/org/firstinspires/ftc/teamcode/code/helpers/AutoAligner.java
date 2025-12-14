package org.firstinspires.ftc.teamcode.code.helpers;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.code.parts.Turret;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;

import java.util.List;



@Config
public class AutoAligner {
    public static final double g = 386.08858; // in/s^2
    public static final double rhinoWheelRadius = 1.88976; // in
    // TODO: Tune these values
    public static final double motorTicksPerRev = 28d;
    public static double kSlip = 0;
    public static double hoodAngle = 0;
    public static double aprilAlignOffset = 0;
    public static double kSlipTurretRotationConstant = 0.01;
    public static double autoAlignBuffer = 5; // degrees
    public static double angleOffsetMult = 1;

    public static double maxShooterVelocity = 2800;
    public static double goalX;
    public static double goalY;
    public static double redGoalX = 70;
    public static double redGoalY = -70;
    public static double blueGoalX = 70;
    public static double blueGoalY = 70;
    public static double aprilX;
    public static double aprilY;
    public static double goalDZ = 28;
    public double angleOffset = 0;
    private int targetAprilID;
    private int sideFlipMultiplier;
    private final OdometryHolonomicDrivetrain driveTrain;
    private final Turret turret;
    private final Limelight3A limelight;

    // 1. kSlip, 2. hoodAngle, 3.
    public static class PointValues {
        public double x, y;
        public double[] v;

        public PointValues(double x, double y, double[] v) {
            this.x = x;
            this.y = y;
            this.v = v;
        }
    }

    private final PointValues[] interpolationPoints = {
//            new PointValues(-48, 0, new double[]{0.44, 50, 0}),
//            new PointValues(-64, 32, new double[]{0.425, 50, 1}),
//            new PointValues(-64, -32, new double[]{0.42, 50, 0}),
//            new PointValues(-55, 17, new double[]{0.435, 50, 1}),
//            new PointValues(-55, -17, new double[]{0.425, 50, 1}),
              new PointValues(-45, 0, new double[]{0.47, 50, 1.5}),
              new PointValues(-54, 18, new double[]{0.47, 50, 1}),
              new PointValues(-54, -18, new double[]{0.47, 50, 1}),
              new PointValues(-63, 33, new double[]{0.47, 50, 3}),

              new PointValues(25, 25, new double[]{0.5, 44, 0}),
              new PointValues(14, 14, new double[]{0.52, 50, 0}),

//              new PointValues(24, 24, new double[]{0.49, 40, 0}),
              new PointValues(0, 0, new double[]{0.51, 50, 0}),
              new PointValues(36, 36, new double[]{0.475, 30, 0}),
              new PointValues(24, -24, new double[]{0.49, 50, -0.5}),
              new PointValues(48, 0, new double[]{0.51, 50, -1.5}),
              new PointValues(36, -36, new double[]{0.49, 50, -2.5})
//
//            new PointValues(0, 0, new double[]{0.45, 50, -2}),
//            new PointValues(39, 39, new double[]{0.425, 27, -1}),
//            new PointValues(39, -39, new double[]{0.435, 50, -2}),
//            new PointValues(48, 0, new double[]{0.47, 50, -2}),
    };

    public AutoAligner(OdometryHolonomicDrivetrain driveTrain, Turret turret, Limelight3A limelight, boolean isRed) {
        this.driveTrain = driveTrain;
        this.turret = turret;
        this.limelight = limelight;

        if (isRed) {
            setRed();
        } else {
            setBlue();
        }
    }

    public void setBlue() {
        goalX = blueGoalX;
        goalY = blueGoalY;
        aprilX = 60;
        aprilY = 54;
        targetAprilID = 20;
        sideFlipMultiplier = 1;
    }

    public void setRed() {
        goalX = redGoalX;
        goalY = redGoalY;
        aprilX = 60;
        aprilY = -54;
        targetAprilID = 24;
        sideFlipMultiplier = -1;
    }

    public double getDistanceToGoal() {
        LLResult result = limelight.getLatestResult();
        Pose2D pos = driveTrain.getPosition();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();
            for (LLResultTypes.FiducialResult tag : aprilTags) {
                if (tag.getFiducialId() == targetAprilID) {
                    Position robotPose = tag.getRobotPoseFieldSpace().getPosition();
                    Log.d("Limelight", "using limelight dist");
                    if (Math.hypot(robotPose.x * 39.37 + pos.getX(DistanceUnit.INCH), robotPose.y * 39.37 + pos.getY(DistanceUnit.INCH)) > 30) {
                        break;
                    }
                    return Math.hypot(goalY + robotPose.y * 39.37, goalX + robotPose.x * 39.37);
                }
            }
        }
        Log.d("Limelight", "Pinpoint dist");
        return Math.hypot(goalX - pos.getX(DistanceUnit.INCH), goalY - pos.getY(DistanceUnit.INCH));
    }

    public double getDistanceToApril() {
        Pose2D pos = driveTrain.getPosition();
        return Math.hypot(aprilX - pos.getX(DistanceUnit.INCH), aprilY - pos.getY(DistanceUnit.INCH));
    }

    public double getRawTurretAutoAlignAngle() {
        Pose2D pos = driveTrain.getPosition();
        double turretAngle = turret.getTurretCurrentAngle();
        LLResult result = limelight.getLatestResult();
//        double pinpointAngle = normalize(180 + Math.toDegrees(Math.atan2(
//                goalY - pos.getY(DistanceUnit.INCH),
//                goalX - pos.getX(DistanceUnit.INCH)
//        )) - pos.getHeading(AngleUnit.DEGREES));
//        Log.d("Turret Debug", "Pinpoint wanted: " + pinpointAngle);
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();
            for (LLResultTypes.FiducialResult tag : aprilTags) {
                if (tag.getFiducialId() == targetAprilID) {
                    // Testing making it aim not directly at apriltag, doesnt completely work
//                    Position robotPose = tag.getRobotPoseFieldSpace().getPosition();
//                    Log.d("Limelight", "See tag");
//                    Log.d("Limelight", "Pose: " + robotPose);
////                    driveTrain.setPosition(new Pose2D(DistanceUnit.METER, -robotPose.x, -robotPose.y, AngleUnit.DEGREES, pos.getHeading(AngleUnit.DEGREES)));
//                    if (Math.hypot(robotPose.x * 39.37 + pos.getX(DistanceUnit.INCH), robotPose.y * 39.37 + pos.getY(DistanceUnit.INCH)) > 30) {
////                        Log.d("Limelight", "Pos off");
//                        break;
//                    }
//                    return normalize(180 + Math.toDegrees(Math.atan2(goalY + robotPose.y * 39.37, goalX + robotPose.x * 39.37)) - pos.getHeading(AngleUnit.DEGREES));

                    // Return limelight angle if it sees the tag
//                    double limelightAngle = turretAngle - tag.getTargetXDegrees();
//                    Log.d("Turret Debug", "Limelight wanted: " + limelightAngle);
//                    return limelightAngle;
                    return turretAngle - tag.getTargetXDegrees() + aprilAlignOffset + angleOffset;
                }
            }
        }
        Log.d("Limelight", "Pinpoint fallback");
        // Fallback on using pinpoint
        return normalize(180 + Math.toDegrees(Math.atan2(
                        goalY - pos.getY(DistanceUnit.INCH),
                        goalX - pos.getX(DistanceUnit.INCH)
                )) - pos.getHeading(AngleUnit.DEGREES));
//        return pinpointAngle;
    }

    public double getTurretAutoAlignAngle() {
        double rawAngle = getRawTurretAutoAlignAngle();
        return Range.clip(rawAngle, turret.MIN_ANGLE_LIMIT, turret.MAX_ANGLE_LIMIT);
    }

    public double getDrivetrainAutoAlignAngle() {
        double rawTurretAngle = getRawTurretAutoAlignAngle();
        double drivetrainAngle = driveTrain.getPosition().getHeading(AngleUnit.DEGREES);
        if (rawTurretAngle >= turret.MIN_ANGLE_LIMIT && rawTurretAngle <= turret.MAX_ANGLE_LIMIT) {
            return drivetrainAngle;
        }
        if (rawTurretAngle < turret.MIN_ANGLE_LIMIT) {
            return normalize(drivetrainAngle + rawTurretAngle - turret.MIN_ANGLE_LIMIT - autoAlignBuffer);
        } else {
            return normalize(drivetrainAngle + rawTurretAngle - turret.MAX_ANGLE_LIMIT + autoAlignBuffer);
        }
    }

    public void updateInterpolation(double x, double y) {
        int dims = interpolationPoints[0].v.length;       // number of value components
        double[] weighted = new double[dims];
        double totalWeight = 0;
        double eps = 1e-9;

        for (PointValues p : interpolationPoints) {
            double dx = x - p.x;
            double dy = y - p.y * sideFlipMultiplier;
            double distSq = dx * dx + dy * dy;

            // Exact match
            if (distSq < 1e-12) {
                kSlip = p.v[0];
                hoodAngle = p.v[1];
                aprilAlignOffset = p.v[2] * sideFlipMultiplier;
            }

            double w = 1.0 / (distSq + eps); // weight = 1/d^2

            for (int i = 0; i < dims; i++) {
                weighted[i] += w * p.v[i];
            }
            totalWeight += w;
        }

        // Normalize by weight sum
        for (int i = 0; i < dims; i++) {
            weighted[i] /= totalWeight;
        }

        kSlip = weighted[0];
        hoodAngle = weighted[1];
        aprilAlignOffset = weighted[2] * sideFlipMultiplier;
    }

    // TODO: Check angle and velocity calculations
    // Distances should be passed in as inches due to FTC standard units
    // theta is the launch angle in degrees, where launching straight up is 0 degrees (hood flat) and straight forward is 90 degrees (hood vertical)
    // Returns -1 if no valid solution (i.e. not possible given the angle)
    public double getShooterVelocityFromAngle(double theta) {
        double horizontalDist = getDistanceToGoal();

        double adjustedKSlip = kSlip + kSlipTurretRotationConstant * Math.abs(turret.getTurretCurrentAngle()) / 180;
        // Convert theta to radians
        theta = Math.toRadians(theta);
        // Impossible
        if (goalDZ >= horizontalDist / Math.tan(theta) || theta <= 0 || horizontalDist <= 0) {
            return -100;
        }
        // First calculate the required launch velocity (derived from kinematics assuming no air resistance)

        Pose2D pos = driveTrain.getPosition();

        double dx = goalX - pos.getX(DistanceUnit.INCH);
        double dy = goalY - pos.getY(DistanceUnit.INCH);
        double dist = Math.hypot(dx, dy);

        double vx = driveTrain.getXVelocity();
        double vy = driveTrain.getYVelocity();

        // towards/away from goal
        double velPar = 0.0;

        // sideways/strafing around goal
        double velPerp = 0.0;

        if (dist != 0) {
            double ux = dx / dist;
            double uy = dy / dist;

            velPar = vx * ux + vy * uy;
            velPerp = vx * uy - vy * ux;
        }

        double sin = Math.sin(theta);
        double cos = Math.cos(theta);

        double a = -goalDZ * sin * sin + cos * sin * horizontalDist;
        double b = -2 * goalDZ * sin * velPar + cos * velPar * horizontalDist;
        double c = (-g * horizontalDist * horizontalDist / 2.0) - goalDZ * velPar * velPar;

        double discriminant = b * b - 4 * a * c;
        double v = (-b + Math.sqrt(discriminant)) / (2 * a);

//        angleOffset = Math.atan2(-velPerp, v * sin + velPar);
        angleOffset = -angleOffsetMult * Math.toDegrees(Math.asin(-velPerp / (v * sin)));


//        double v = Math.sqrt(
//                g * Math.pow(horizontalDist, 2) /
//                        (2 * Math.pow(Math.sin(theta), 2) * (horizontalDist / Math.tan(theta) - goalDZ))
//        );

        // Then calculate RPM from linear velocity
        double rpm = (v * 60) / (2 * Math.PI * rhinoWheelRadius * adjustedKSlip);
        // Scale to motor ticks per second
        // TODO: Check ticks/sec calculation
        double rawTPS = rpm * motorTicksPerRev / 60;
        if (rawTPS > maxShooterVelocity) {
            Log.d("Above Max Shooter", "raw vel: " + rawTPS);
        }
        return Range.clip(rpm * motorTicksPerRev / 60, 0, maxShooterVelocity);
    }

    public double getOptimalHoodAngle() {
        return hoodAngle;
    }

    // Work in progress
    public double getHoodAngleFromVelocity(double v, boolean useNegSol) {
        // All inputs must be in inches and inches/second
        double x = getDistanceToGoal();
        double xSquared = x * x;
        double vSquared = v * v;
        double gTerm = g * xSquared / (2 * vSquared);

        double discriminant = xSquared - (4 * gTerm * (goalDZ + gTerm));

        if (discriminant < 0) {
            return Double.NaN;
        }

        double sol = useNegSol ? (x - Math.sqrt(discriminant)) / (2 * gTerm) : (x + Math.sqrt(discriminant)) / (2 * gTerm);

        return 90 - Math.toDegrees(Math.atan(sol));
    }

    public double getHoodAngleFromVelocity(double v) {
        return getHoodAngleFromVelocity(v, false);
    }

    // Work in progress
    public double getOptimalShooterVelocity() {
        return Math.min(900 + (getDistanceToGoal() / 156) * 900, 1800);
    }

    private double normalize(double angle) {
        return (angle + 180) % 360 - 180;
    }
}
