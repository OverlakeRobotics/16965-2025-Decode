// Interface for all odometry modules that get the robots position in some way.

package org.firstinspires.ftc.teamcode.system;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public interface OdometryModule {
    // Behavior: Gets the position of the robot according to last reading of the odometry sensor.
    //           The y value is forward from the starting position and the x value is to the right.
    // Returns: A Pose2D object containing the robots position.
    Pose2D getPosition();

    // Behavior: Reads the values from the odometry sensor and updates what it thinks the current
    //           position is.
    void updatePosition();

    // Behavior: Sets the current position of the odometry sensor to the given position.
    // Parameters:
    //      - Pose2D position: The position to set the odometry sensor to.
    void setPosition(Pose2D position);

    // Behavior: Gets the velocity of the bot in the x direction.
    // Returns: The velocity of the bot in the x direction.
    default double getXVelocity() {
        throw new UnsupportedOperationException("X velocity not supported");
    }

    // Behavior: Gets the velocity of the bot in the y direction.
    // Returns: The velocity of the bot in the y direction.
    default double getYVelocity() {
        throw new UnsupportedOperationException("Y velocity not supported");
    }

    // Behavior: Gets the angular velocity of the bot.
    // Returns: The angular velocity of the bot in radians/s.
    default double getAngularVelocity() {
        throw new UnsupportedOperationException("Angular velocity not supported");
    }

    // Behavior: Sets the priority level for the position reading of the odometry sensor. This is
    //           used to determine which sensors readings to use when using multiple odometry
    //           sensors to determine the robots position. A higher priority means it will be
    //           used over other sensors with lower priority levels. With the same priority, it
    //           takes whichever comes first.
    // Parameters:
    //      - int priority: The priority level of the odometry sensor.
    void setPositionPriority(int priority);

    // Behavior: Gets the priority level for the odometry levels position.
    // Returns: The priority level for the odometry sensors position readings, an integer.
    int getPositionPriority();

    // Behavior: Sets the priority level for the heading reading of the odometry sensor. This is
    //           used to determine which sensors readings to use when using multiple odometry
    //           sensors to determine the robots heading. A higher priority means it will be
    //           used over other sensors with lower priority levels. With the same priority, it
    //           takes whichever comes first.
    // Parameters:
    //      - int priority: The priority level of the odometry sensor.
    void setHeadingPriority(int priority);

    // Behavior: Gets the priority level for the odometry levels heading.
    // Returns: The priority level for the odometry sensors heading readings, an integer.
    int getHeadingPriority();

    // Behavior: Sets whether the odometry sensor should set its position to the position readings of
    //           a sensor with a higher position priority that was able to update its position.
    // Parameters:
    //      - boolean doReset: Whether it should reset its position to higher priority sensors.
    void setDoPositionResetToHigherPriority(boolean doReset);

    // Behavior: Returns whether the sensor will reset its position to higher priority sensors.
    // Returns: A boolean of whether or not it will reset its position to higher priority sensors.
    boolean doPositionResetToHigherPriority();

    // Behavior: Sets whether the odometry sensor should set its heading to the heading readings of
    //           a sensor with a higher heading priority that was able to update its heading.
    // Parameters:
    //      - boolean doReset: Whether it should reset its heading to higher priority sensors.
    void setDoHeadingResetToHigherPriority(boolean doReset);

    // Behavior: Returns whether the sensor will reset its heading to higher priority sensors.
    // Returns: A boolean of whether or not it will reset its heading to higher priority sensors.
    boolean doHeadingResetToHigherPriority();

    // Behavior: Tells you whether or not the position measurements of the sensor are currently
    //           accurate.
    boolean isPositionAccurate();

    // Behavior: Tells you whether or not the heading measurements of the sensor are currently
    //           accurate.
    boolean isHeadingAccurate();

    // Behavior: Resets the odometry sensors heading and position to 0.
    void reset();
}
