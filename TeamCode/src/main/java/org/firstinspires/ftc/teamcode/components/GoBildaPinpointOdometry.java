// Implementation of the OdometryModule interface for SparkFunOTOS sensor.

package org.firstinspires.ftc.teamcode.components;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.system.OdometryModule;

public class GoBildaPinpointOdometry implements OdometryModule {
    private final GoBildaPinpointDriver pinpoint;
    private Pose2D position;
    private int positionPriority;
    private int headingPriority;
    private boolean doPositionReset;
    private boolean doHeadingReset;
    public GoBildaPinpointOdometry(GoBildaPinpointDriver pinpoint) {
        this.pinpoint = pinpoint;

        positionPriority = 3;
        headingPriority = 2;
        doPositionReset = true;
        doHeadingReset = false;
    }

    public Pose2D getPosition() {
        return position;
    }

    public void updatePosition() {
        pinpoint.update();
        position = pinpoint.getPosition();
    }

    public void setPosition(Pose2D position) {
        pinpoint.setPosition(position);
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
        return true;
    }

    public boolean isHeadingAccurate() {
        return true;
    }

    public void reset() {
        pinpoint.resetPosAndIMU();
    }
}
