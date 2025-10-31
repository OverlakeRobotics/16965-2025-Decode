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
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;

import java.util.List;

@Config
public class AutoAligner {
    public static final double g = 386.08858; // in/s^2
    public static final double rhinoWheelRadius = 1.88976; // in
    // TODO: Tune this value
    public static double k_slip = 0.42; // estimated slip factor
    public static final double motorTicksPerRev = 28d;
    public static double maxShooterVelocity = 1800;
    public static double angleMin = 5;
    public static double angleMax = 35;
    private double goalX;
    private double goalY;
    private double aprilX;
    private double aprilY;
    private double goalDZ = 28;
    private int targetAprilID;
    private OdometryHolonomicDrivetrain driveTrain;
    private Limelight3A limelight;


    public AutoAligner(OdometryHolonomicDrivetrain driveTrain, Limelight3A limelight, boolean isRed) {
        this.driveTrain = driveTrain;
        this.limelight = limelight;
        if (isRed) {
            setRed();
        } else {
            setBlue();
        }
    }

    public void setBlue() {
        goalX = 70;
        goalY = 65;
        aprilX = 60;
        aprilY = 54;
        targetAprilID = 20;
    }

    public void setRed() {
        goalX = 70;
        goalY = -65;
        aprilX = 60;
        aprilY = -54;
        targetAprilID = 23;
    }

    public double getDistanceToGoal() {
        Pose2D pos = driveTrain.getPosition();
        return Math.hypot(goalX - pos.getX(DistanceUnit.INCH), goalY - pos.getY(DistanceUnit.INCH));
    }

    public double getDistancetoApril() {
        Pose2D pos = driveTrain.getPosition();
        return Math.hypot(aprilX - pos.getX(DistanceUnit.INCH), aprilY - pos.getY(DistanceUnit.INCH));
    }

    public double getAutoAlignAngle() {
        double distance = getDistanceToGoal();
        Pose2D pos = driveTrain.getPosition();
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();
            for (LLResultTypes.FiducialResult tag : aprilTags) {
                if (tag.getFiducialId() == targetAprilID) {
                    double aprilAngle = driveTrain.getPosition().getHeading(AngleUnit.DEGREES) - tag.getTargetXDegrees();
                    return aprilAngle - Math.atan2(goalX - (aprilX - distance * Math.cos(aprilAngle)), goalY - (aprilY - distance * Math.sin(aprilAngle)));
                }
            }
        }

        return Math.toDegrees(Math.atan2(
                goalY - pos.getY(DistanceUnit.INCH),
                goalX - pos.getX(DistanceUnit.INCH)
        ));
    }

    // TODO: Check angle and velocity calculations
    // Distances should be passed in as inches due to FTC standard units
    // theta is the launch angle in degrees, where launching straight up is 0 degrees (hood flat) and straight forward is 90 degrees (hood vertical)
    // Returns -1 if no valid solution (i.e. not possible given the angle)
    public double getShooterVelocityFromAngle(double theta) {
        double horizontalDist = getDistanceToGoal();
        // Convert theta to radians
        theta = Math.toRadians(theta);
        // Impossible
        if (goalDZ >= horizontalDist / Math.tan(theta) || theta <= 0 || horizontalDist <= 0) {
            return -100;
        }
        // First calculate the required launch velocity (derived from kinematics assuming no air resistance)
        double v = Math.sqrt(
                g * Math.pow(horizontalDist, 2) /
                        (2 * Math.pow(Math.sin(theta), 2) * (horizontalDist / Math.tan(theta) - goalDZ))
        );

        // Then calculate RPM from linear velocity
        double rpm = (v * 60) / (2 * Math.PI * rhinoWheelRadius * k_slip);
        // Scale to motor ticks per second
        // TODO: Check ticks/sec calculation
        double rawTPS = rpm * motorTicksPerRev / 60;
        if (rawTPS > maxShooterVelocity) {
            Log.d("Above Max Shooter", "raw vel: " + rawTPS);
        }
        return Range.clip(rpm * motorTicksPerRev / 60, 0, maxShooterVelocity);
    }

    public double getOptimalHoodAngle() {
        return Math.min(angleMin + (getDistanceToGoal() / 144) * (angleMax - angleMin), angleMax);
    }

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

    public double getOptimalShooterVelocity() {
        return Math.min(900 + (getDistanceToGoal() / 156) * 900, 1800);
    }
}
