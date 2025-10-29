// Extension of the BasicHolonomicDrivetrain class adding the ability to use odometry. This adds
// features like heading correction, turning to a specific angle, field centric driving, moving to
// specific positions, and driving along a predefined path.
// Coordinate system used by this class: Positive x is forward, positive y is left, and
// positive heading is turning to the left.

package org.firstinspires.ftc.teamcode.system;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class OdometryHolonomicDrivetrain extends BasicHolonomicDrivetrain {
    private static final double P_GAIN = 50;
    private static final double MIN_DIST_TO_STOP = 0.5;
    private static final double COUNTS_PER_DEGREE = 10;
    private static final double MIN_ANGLE_DIF_TO_STOP = 1;
    private double pathTolerance = 4;
    private boolean doPositionHeadingCorrection;
    private boolean positionDriveUsingOdometry;
    private final OdometryModule odometry;
    private double lastHeading;
    private Pose2D currentPosition;
    private Pose2D wantedPosition;
    private double positionDriveDirection;
    private Pose2D[] currentPath;
    private double[] pathDistances;
    private int currentPoint = -1;

    public OdometryHolonomicDrivetrain(DcMotorEx backLeft, DcMotorEx backRight, DcMotorEx frontLeft,
                                       DcMotorEx frontRight, OdometryModule odometry) {
        super(backLeft, backRight, frontLeft, frontRight);
        this.odometry = odometry;
        this.odometry.updatePosition();
        this.currentPosition = this.odometry.getPosition();
        this.wantedPosition = currentPosition;

        lastHeading = this.currentPosition.getHeading(AngleUnit.DEGREES);
    }
    
    // Behavior: Overrides super classes drive method to include odometry and heading correction.
    @Override
    public void drive() {
        switch (currentDriveState) {
            case STOPPED:
            case VELOCITY_DRIVE:
                break;

            case POSITION_DRIVE:
                if (currentPoint >= 0) {
                    boolean atNextPoint = true;
                    while (atNextPoint) {
                        if ((currentPoint != currentPath.length - 1) && getDistanceToDestination() < pathTolerance) {
                            currentPoint++;
                            wantedPosition = currentPath[currentPoint];
                        } else {
                            atNextPoint = false;
                        }
                    }

                    int nextPoint = currentPoint;
                    setPositionDrive(currentPath[nextPoint], forward);
                    currentPoint = nextPoint;
                } else if (positionDriveUsingOdometry) {
                    setPositionDrive(wantedPosition, forward);
                } else if (doPositionHeadingCorrection) {
                    setPositionDriveCorrection(getPositionDriveDistanceLeft(), positionDriveDirection, forward, wantedPosition.getHeading(AngleUnit.DEGREES));
                } else {
                    // TODO: Check if this is right, because its using positionDriveDirection, but that is only set if doPositionHeadingCorrection is true.
                    super.setPositionDrive(getPositionDriveDistanceLeft(), positionDriveDirection - currentPosition.getHeading(AngleUnit.DEGREES), forward);
                }
                break;
        }

        lastHeading = currentPosition.getHeading(AngleUnit.DEGREES);
        super.drive();
    }

    // Behavior: Sets the velocity of the robot while accounting for a field centric view.
    //           This means when that the forward value causes the robot to move in the 0 degree
    //           direction, no matter which way the robot is facing. Similarly, the strafe value
    //           will cause the robot to move in the 90 degree direction.
    public void setVelocityDriveFieldCentric(double forward, double strafe, double turn) {
        double heading = currentPosition.getHeading(AngleUnit.RADIANS);
        super.setVelocityDrive(
            forward * Math.cos(heading) + strafe * Math.sin(heading),
            strafe * Math.cos(heading) - forward * Math.sin(heading),
            turn
        );
    }

    // Behavior: Overrides basic setPositionDrive so whenever a position drive is set, the state
    //           of the current position drive is reset.
    @Override
    public void setPositionDrive(int backLeftTarget, int backRightTarget, int frontLeftTarget, int frontRightTarget, double velocity) {
        super.setPositionDrive(backLeftTarget, backRightTarget, frontLeftTarget, frontRightTarget, velocity);
        doPositionHeadingCorrection = false;
        positionDriveUsingOdometry = false;
        currentPoint = -1;
    }

    // Behavior: Method just like setPosition drive but also includes a wantedH for heading correction.
    public void setPositionDriveCorrection(int distance, double direction, double velocity, double wantedH) {
        double dh = normalize(currentPosition.getHeading(AngleUnit.DEGREES) - lastHeading);
        double angleOffset = dh / 2;
        // TODO: Test angle offset by driving in a straight line.
        super.setPositionDrive(distance, direction - currentPosition.getHeading(AngleUnit.DEGREES) - angleOffset,
                COUNTS_PER_DEGREE * normalize(wantedH - currentPosition.getHeading(AngleUnit.DEGREES)), velocity);
        positionDriveDirection = direction;
        setWantedHeading(wantedH);
        doPositionHeadingCorrection = true;
    }

    // Behavior: Overloads the setPositionDrive function to go to a specified location using odometry.
    // Parameters:
    //      - Pose2D wantedPosition: The wanted position (x, y, heading) of the robot.
    //      - double velocity: How fast the robot will move to the wanted position, in counts/s
    public void setPositionDrive(Pose2D wantedPosition, double velocity) {
        this.wantedPosition = wantedPosition;
        double dx = (this.wantedPosition.getX(DistanceUnit.INCH) - this.currentPosition.getX(DistanceUnit.INCH));
        double dy = (this.wantedPosition.getY(DistanceUnit.INCH) - this.currentPosition.getY(DistanceUnit.INCH));
        // TODO: Make not just FORWARD_COUNTS_PER_INCH but a combination of strafe and forward counts per inch.
        //       The robot may be rotated so dx isn't just forward and dy isnt just strafe.
        int counts = (int) Math.round(Math.hypot(dx * FORWARD_COUNTS_PER_INCH, dy * FORWARD_COUNTS_PER_INCH));
        double direction = Math.toDegrees(Math.atan2(dy, dx));
        setPositionDriveCorrection(counts, direction, velocity, wantedPosition.getHeading(AngleUnit.DEGREES));
        positionDriveUsingOdometry = true;
    }

    // Behavior: Has the robot drive along a path defined by a list of sequential points. The tolerance
    //           for how close it has to stick to this path is defined by pathTolerance. A higher value
    //           will give less jitter, and a lower value will give more accuracy.
    // Parameters:
    //      - Pose2D[] path: The path for the robot to follow as an array of Pose2Ds.
    //      - double velocity: How fast the robot should follow the path.
    public void setPositionDrive(Pose2D[] path, double velocity, int initialPointIndex) {
        double[] distances = new double[path.length];
        for (int i = path.length - 2; i >= 0; i--) {
            distances[i] = distances[i + 1] + dist(path[i], path[i + 1]);
        }

        pathDistances = distances;

        currentPath = path;
        setPositionDrive(currentPath[initialPointIndex], velocity);
        currentPoint = initialPointIndex;
    }

    // Behavior: Overloads setPositionDrive to default as 0 as the initial point.
    public void setPositionDrive(Pose2D[] path, double velocity) {
        setPositionDrive(path, velocity, 0);
    }

    // Behavior: Sets the tolerance parameter for driving along a path.
    // Parameters:
    //      - double tolerance: How far the robot can deviate from the path in inches. A lower tolerance
    //                          will make the robot slower and jittery but more accurate, while a
    //                          tolerance that is higher will be smoother but less accurate.
    public void setTolerance(double tolerance) {
        pathTolerance = tolerance;
    }

    // Behavior: Gets the distance from the current position to the wanted position.
    // Returns: A double, in inches, containing the distance to the destination.
    public double getDistanceToDestination() {
        return Math.hypot(wantedPosition.getX(DistanceUnit.INCH) - currentPosition.getX(DistanceUnit.INCH),
                          wantedPosition.getY(DistanceUnit.INCH) - currentPosition.getY(DistanceUnit.INCH));
    }


    // Behavior: Overrides getPositionDriveDistanceLeft to make the distance count the entire path's
    //           length when doing a path drive.
    @Override
    public int getPositionDriveDistanceLeft() {
        if (currentPoint >= 0) {
            double totalDist = pathDistances[currentPoint] + dist(currentPosition, currentPath[currentPoint]);
            return (int)(totalDist * FORWARD_COUNTS_PER_INCH);
        }

        return super.getPositionDriveDistanceLeft();
    }

    @Override
    public boolean isDriving() {
        if (currentPoint >= 0) {
            return !((currentPoint == currentPath.length - 1) && (getDistanceToDestination() < MIN_DIST_TO_STOP && Math.abs(getTurnCountsLeft() / COUNTS_PER_DEGREE) < MIN_ANGLE_DIF_TO_STOP));
        } else if (positionDriveUsingOdometry) {
            return !(getDistanceToDestination() < MIN_DIST_TO_STOP && Math.abs(getTurnCountsLeft() / COUNTS_PER_DEGREE) < MIN_ANGLE_DIF_TO_STOP);
        }
        return super.isDriving();
    }

    // Behavior: Sets the wanted heading of the robot.
    // Parameters:
    //      - double wantedHeading: The wanted heading of the robot in degrees.
    public void setWantedHeading(double wantedHeading) {
        wantedPosition = new Pose2D(DistanceUnit.INCH, wantedPosition.getX(DistanceUnit.INCH),
                wantedPosition.getY(DistanceUnit.INCH), AngleUnit.DEGREES, wantedHeading);
    }

    // Behavior: Gets the turn velocity needed to heading correct the robot.
    // Returns: The turn velocity.
    public double getHeadingCorrectionVelocity() {
        return P_GAIN * normalize(normalize(wantedPosition.getHeading(AngleUnit.DEGREES)) -
                         normalize(currentPosition.getHeading(AngleUnit.DEGREES)));
    }

    // Behavior: Updates the position of the robot. This needs to be called in the loop for accurate
    //           positioning.
    public void updatePosition() {
        odometry.updatePosition();
        currentPosition = odometry.getPosition();
    }

    // Behavior: Gets whether the robot is stopped.
    // Returns: Whether or not the current drive state is stopped.
    public boolean isStopped() {
        return currentDriveState == DriveState.STOPPED;
    }

    // Behavior: Gets the current position of the robot.
    // Returns: A Pose2D with the position of the robot, x, y, and heading.
    public Pose2D getPosition() {
        return currentPosition;
    }

    // Behavior: Sets the position of the robot. For example, if you set the position to (10, -10, 45),
    //           the robot will think its current x and y are (10, -10) and its heading is 45 degrees.
    // Parameters:
    //      - Pose2D position: The position to set the robot to.
    public void setPosition(Pose2D position) {
        odometry.setPosition(position);
    }

    // Behavior: Normalizes an angle to (-180,180]
    // Returns: The normalized angle
    private static double normalize(double degrees) {
        double normalizedAngle = degrees;
        while (normalizedAngle > 180) normalizedAngle -= 360;
        while (normalizedAngle <= -180) normalizedAngle += 360;
        return normalizedAngle;
    }

    // Behavior: Gets the distance between two Pose2Ds.
    // Returns: The distance, in inches.
    public static double dist(Pose2D p1, Pose2D p2) {
        return Math.hypot(
            (p1.getX(DistanceUnit.INCH) - p2.getX(DistanceUnit.INCH)),
            (p1.getY(DistanceUnit.INCH) - p2.getY(DistanceUnit.INCH))
        );
    }

    // Behavior: Returns the index of the current point if path driving, otherwise -1.
    // Returns: The index of the current point in the path, or -1 if not path driving.
    public int getNextPointIndex() {
        return currentPoint;
    }
}
