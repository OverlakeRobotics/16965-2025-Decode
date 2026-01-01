package org.firstinspires.ftc.teamcode.system;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;


public class OdometryCollection implements OdometryModule {
    List<OdometryModule> odometryModules;
    Pose2D position;
    private int positionPriority;
    private int headingPriority;
    private boolean doPositionReset;
    private boolean doHeadingReset;
    private boolean isPositionAccurate;
    private boolean isHeadingAccurate;
    public OdometryCollection(List<OdometryModule> odometryModules) {
        this.odometryModules = odometryModules;
        positionPriority = Integer.MAX_VALUE;
        headingPriority = Integer.MAX_VALUE;
        for (OdometryModule module : odometryModules) {
            if (module.getPositionPriority() < positionPriority) {
                positionPriority = module.getPositionPriority();
            }

            if (module.getHeadingPriority() < headingPriority) {
                headingPriority = module.getHeadingPriority();
            }
        }

        doPositionReset = false;
        doHeadingReset = false;
    }

    // Behavior: Updates the position of all odometry modules taking into account their priorities.
    public void updatePosition() {
        for (OdometryModule module : odometryModules) {
            module.updatePosition();
        }

        Pose2D bestPosition = null;
        int bestPositionPriority = Integer.MAX_VALUE;
        OdometryModule bestPositionModule = null;
        Pose2D bestHeading = null;
        int bestHeadingPriority = Integer.MAX_VALUE;
        OdometryModule bestHeadingModule = null;

        for (OdometryModule module : odometryModules) {
            if (module.isPositionAccurate() && module.getPositionPriority() < bestPositionPriority) {
                bestPosition = module.getPosition();
                bestPositionPriority = module.getPositionPriority();
                bestPositionModule = module;
            }

            if (module.isHeadingAccurate() && module.getHeadingPriority() < bestHeadingPriority) {
                bestHeading = module.getPosition();
                bestHeadingPriority = module.getHeadingPriority();
                bestHeadingModule = module;
            }
        }

        if (bestPosition == null) {
            isPositionAccurate = false;
            for (OdometryModule module : odometryModules) {
                if (module.getPositionPriority() < bestPositionPriority) {
                    bestPosition = module.getPosition();
                    bestPositionPriority = module.getPositionPriority();
                    bestPositionModule = module;
                }
            }
        } else {
            isPositionAccurate = true;
        }

        if (bestHeading == null) {
            isHeadingAccurate = false;
            for (OdometryModule module : odometryModules) {
                if (module.getHeadingPriority() < bestHeadingPriority) {
                    bestHeading = module.getPosition();
                    bestHeadingPriority = module.getHeadingPriority();
                    bestHeadingModule = module;
                }
            }
        } else {
            isHeadingAccurate = true;
        }

        for (OdometryModule module : odometryModules) {
            if (module.doPositionResetToHigherPriority() && bestPosition != null &&
                    bestPositionModule != module) {
                Pose2D modulePosition = module.getPosition();
                module.setPosition(new Pose2D(DistanceUnit.INCH, bestPosition.getX(DistanceUnit.INCH),
                        bestPosition.getY(DistanceUnit.INCH), AngleUnit.DEGREES,
                        modulePosition.getHeading(AngleUnit.DEGREES)));
            }

            if (module.doHeadingResetToHigherPriority() && bestHeading != null && bestHeadingModule != module) {
                Pose2D modulePosition = module.getPosition();
                module.setPosition(new Pose2D(DistanceUnit.INCH, modulePosition.getX(DistanceUnit.INCH),
                        modulePosition.getY(DistanceUnit.INCH), AngleUnit.DEGREES,
                        bestHeading.getHeading(AngleUnit.DEGREES)));
            }
        }

        if (bestPosition != null && bestHeading != null) {
            position = new Pose2D(DistanceUnit.INCH, bestPosition.getX(DistanceUnit.INCH),
                    bestPosition.getY(DistanceUnit.INCH), AngleUnit.DEGREES,
                    bestHeading.getHeading(AngleUnit.DEGREES));
        }
    }

    // Behavior: Gets the current position taking into account all odometry sensors and their priorities.
    // Returns: A Pose2D containing the current position.
    public Pose2D getPosition() {
        return position;
    }

    // Behavior: Sets the position of all odometry modules to the given position.
    // Parameters:
    //      - Pose2D position: The position to set the position of the odometry modules to.
    public void setPosition(Pose2D position) {
        for (OdometryModule module : odometryModules) {
            module.setPosition(position);
        }
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
        return isPositionAccurate;
    }

    public boolean isHeadingAccurate() {
        return isHeadingAccurate;
    }

    // Behavior: Resets all odometry modules positions.
    public void reset() {
        for (OdometryModule module : odometryModules) {
            module.reset();
        }
    }
}
