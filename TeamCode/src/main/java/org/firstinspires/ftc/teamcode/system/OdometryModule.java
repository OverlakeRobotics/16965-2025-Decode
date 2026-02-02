package org.firstinspires.ftc.teamcode.system;


import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


// Interface for all odometry modules that get the robots position in some way.
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

    // Behavior: Resets the odometry sensors heading and position to 0.
    void reset();

    // Behavior: Recalibrates the sensor for this odometry if it has this function.
    default void recalibrate() {
        throw new UnsupportedOperationException("This odometry cannot recalibrate!");
    }

    // Behavior: Gets the velocity of the bot in the x direction.
    // Returns: The velocity of the bot in the x direction.
    default double getXVelocity() {
        throw new UnsupportedOperationException("X velocity not implemented");
    }

    // Behavior: Gets the velocity of the bot in the y direction.
    // Returns: The velocity of the bot in the y direction.
    default double getYVelocity() {
        throw new UnsupportedOperationException("Y velocity not implemented");
    }

    // Behavior: Gets the angular velocity of the bot.
    // Returns: The angular velocity of the bot in radians/s.
    default double getAngularVelocity() {
        throw new UnsupportedOperationException("Angular velocity not implemented");
    }

    // Behavior: Sets the priority level for the position reading of the odometry sensor. This is
    //           used to determine which sensors readings to use when using multiple odometry
    //           sensors to determine the robots position. A higher priority means it will be
    //           used over other sensors with lower priority levels. With the same priority, it
    //           takes whichever comes first.
    // Parameters:
    //      - int priority: The priority level of the odometry sensor.
    default void setPositionPriority(int priority) {
        throw new UnsupportedOperationException("Priority not implemented");
    }

    // Behavior: Gets the priority level for the odometry levels position.
    // Returns: The priority level for the odometry sensors position readings, an integer.
    default int getPositionPriority() {
        throw new UnsupportedOperationException("Priority not implemented");
    }

    // Behavior: Sets the priority level for the heading reading of the odometry sensor. This is
    //           used to determine which sensors readings to use when using multiple odometry
    //           sensors to determine the robots heading. A higher priority means it will be
    //           used over other sensors with lower priority levels. With the same priority, it
    //           takes whichever comes first.
    // Parameters:
    //      - int priority: The priority level of the odometry sensor.
    default void setHeadingPriority(int priority) {
        throw new UnsupportedOperationException("Priority not implemented");
    }

    // Behavior: Gets the priority level for the odometry levels heading.
    // Returns: The priority level for the odometry sensors heading readings, an integer.
    default int getHeadingPriority() {
        throw new UnsupportedOperationException("Priority not implemented");
    }

    // Behavior: Sets whether the odometry sensor should set its position to the position readings of
    //           a sensor with a higher position priority that was able to update its position.
    // Parameters:
    //      - boolean doReset: Whether it should reset its position to higher priority sensors.
    default void setDoPositionResetToHigherPriority(boolean doReset) {
        throw new UnsupportedOperationException("Priority not implemented");
    }

    // Behavior: Returns whether the sensor will reset its position to higher priority sensors.
    // Returns: A boolean of whether or not it will reset its position to higher priority sensors.
    default boolean doPositionResetToHigherPriority() {
        throw new UnsupportedOperationException("Priority not implemented");
    }

    // Behavior: Sets whether the odometry sensor should set its heading to the heading readings of
    //           a sensor with a higher heading priority that was able to update its heading.
    // Parameters:
    //      - boolean doReset: Whether it should reset its heading to higher priority sensors.
    default void setDoHeadingResetToHigherPriority(boolean doReset) {
        throw new UnsupportedOperationException("Priority not implemented");
    }

    // Behavior: Returns whether the sensor will reset its heading to higher priority sensors.
    // Returns: A boolean of whether or not it will reset its heading to higher priority sensors.
    default boolean doHeadingResetToHigherPriority() {
        throw new UnsupportedOperationException("Priority not implemented");
    }

    // Behavior: Tells you whether or not the position measurements of the sensor are currently
    //           accurate.
    default boolean isPositionAccurate() {
        throw new UnsupportedOperationException("Accuracy not implemented");
    }

    // Behavior: Tells you whether or not the heading measurements of the sensor are currently
    //           accurate.
    default boolean isHeadingAccurate() {
        throw new UnsupportedOperationException("Accuracy not implemented");
    }
}
