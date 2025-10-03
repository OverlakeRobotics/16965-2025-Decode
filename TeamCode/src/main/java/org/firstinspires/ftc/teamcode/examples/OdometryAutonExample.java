package org.firstinspires.ftc.teamcode.examples;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;

@Config
@Autonomous(name = "Odometry Auton Example", group = "Autonomous")
public class OdometryAutonExample extends OpMode {
    // Change to your actual offsets. See GoBildaPinpointDriver.setOffsets() for details on measuring offsets.
    public double yOffset = -168.0;
    public double xOffset = -84.0;

    public static int velocity = 2000;

    public static double inchesToChangeDirection = 2.0;
    private OdometryHolonomicDrivetrain driveTrain;

    private final Pose2D[] drivePoints = {
            new Pose2D(DistanceUnit.INCH, 106, 0, AngleUnit.DEGREES, -90),
            new Pose2D(DistanceUnit.INCH, 106, -96, AngleUnit.DEGREES, 180),
            new Pose2D(DistanceUnit.INCH, 14, -96, AngleUnit.DEGREES, 90),
            new Pose2D(DistanceUnit.INCH, 14, 0, AngleUnit.DEGREES, 0),
    };
    private int currentPoint = 0;

    @Override
    public void init() {
        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpointDriver.setOffsets(xOffset, yOffset);
        driveTrain = new OdometryHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                new GoBildaPinpointOdometry(pinpointDriver)
        );
    }

    // This OpMode manually sets the target position of the drive train to the next point after
    // the robot reaches the current target point. This applies slowdown to each point and is mainly
    // used to demonstrate some more of the driveTrain class's capabilities. Practically, you will normally
    // use driveTrain.setPositionDrive(Pose2D[] path, velocity, tolerance) in start() and then just
    // call driveTrain.updatePosition() and driveTrain.drive() in loop().
    @Override
    public void loop() {
        driveTrain.updatePosition();
        driveTrain.drive();

        Pose2D pos = driveTrain.getPosition();
        telemetry.addData("Position", "X: " + pos.getX(DistanceUnit.INCH) + ", Y: " + pos.getY(DistanceUnit.INCH) + ", Heading: " + pos.getHeading(AngleUnit.DEGREES));

        if (driveTrain.getDistanceToDestination() < inchesToChangeDirection) {
            currentPoint++;
            if (currentPoint < drivePoints.length) {
                driveTrain.setPositionDrive(drivePoints[currentPoint], velocity);
            }
        }
    }

    @Override
    public void start() {
        driveTrain.updatePosition();
        driveTrain.setPositionDrive(drivePoints[currentPoint], velocity);
    }
}
