package org.firstinspires.ftc.teamcode.components;


import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.system.OdometryModule;


// Implementation of the OdometryModule interface for the IMU.
public class IMUOdometry implements OdometryModule {
    private final IMU imu;
    private double heading;
    private int positionPriority;
    private int headingPriority;
    private boolean doPositionReset;
    private boolean doHeadingReset;

    public IMUOdometry(IMU imu) {
        this.imu = imu;

        positionPriority = Integer.MAX_VALUE;
        headingPriority = 1;
        doPositionReset = false;
        doHeadingReset = false;
    }

    public void updatePosition() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        heading = orientation.getYaw(AngleUnit.DEGREES);
    }

    public Pose2D getPosition() {
        return new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, heading);
    }

    public void setPosition(Pose2D position) {
        heading = position.getHeading(AngleUnit.DEGREES);
    }

    public void setPositionPriority(int priority) {
        positionPriority = priority;
    }

    public int getPositionPriority() {
        return positionPriority;
    }

    public void setHeadingPriority(int priority) {
        headingPriority = priority;
    }

    public int getHeadingPriority() {
        return headingPriority;
    }

    public void setDoPositionResetToHigherPriority(boolean doReset) {
        doPositionReset = doReset;
    }

    public boolean doPositionResetToHigherPriority() {
        return doPositionReset;
    }

    public void setDoHeadingResetToHigherPriority(boolean doReset) {
        doHeadingReset = doReset;
    }

    public boolean doHeadingResetToHigherPriority() {
        return doHeadingReset;
    }

    public boolean isPositionAccurate() {
        return false;
    }

    public boolean isHeadingAccurate() {
        return true;
    }

    public void reset() {
        imu.resetYaw();
    }
}
