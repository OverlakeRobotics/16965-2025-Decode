package org.firstinspires.ftc.teamcode.examples;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;


@Config
@TeleOp(name = "Odometry TeleOp Example", group = "TeleOp")
public class OdometryTeleOpExample extends OpMode {
    public double yOffset = -168.0; // mm
    public double xOffset = -84.0; // mm

    public double autoLockX = 60;
    public double autoLockY = 54;

    // Positive angle is to the left, positive x is forward, and positive y is left
    // This is the center of the bot when the program is initialized
    public Pose2D startPos = new Pose2D(DistanceUnit.INCH, -63, 15, AngleUnit.DEGREES, 0);

    // List of preset positions the driver can press a button to start driving to
    public Pose2D[] presetPositions = {
            new Pose2D(DistanceUnit.INCH, -54, 0, AngleUnit.DEGREES, 0),
            new Pose2D(DistanceUnit.INCH, 27, 21, AngleUnit.DEGREES, 0),
    };

    public int currentPreset = -1;
    public double velocity = 2000;
    private OdometryHolonomicDrivetrain driveTrain;
    private boolean autoLock = false;

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

        driveTrain.setPosition(startPos);
        driveTrain.setCountsToSlowDown(500);
    }

    @Override
    public void loop() {
        driveTrain.updatePosition();
        Pose2D currentPos = driveTrain.getPosition();

        telemetry.addData("Position", "X: %.2f, Y: %.2f, H: %.2f", currentPos.getX(DistanceUnit.INCH), currentPos.getY(DistanceUnit.INCH), currentPos.getHeading(AngleUnit.DEGREES));

        double wantedHeading = Math.toDegrees(Math.atan2(
                autoLockY - currentPos.getY(DistanceUnit.INCH),
                autoLockX - currentPos.getX(DistanceUnit.INCH)
        ));

        // Set presets if a button was pressed.
        if (gamepad1.a) {
            currentPreset = 0;
        } else if (gamepad1.b) {
            currentPreset = 1;
        }

        if (currentPreset >= 0 && (Math.abs(gamepad1.left_stick_x) > 0.001 || Math.abs(gamepad1.left_stick_y) > 0.001 || Math.abs(gamepad1.right_stick_x) > 0.001)) {
            currentPreset = -1;
            autoLock = true;
        }

        if (currentPreset >= 0) {
            // Drive to preset position if preset is selected
            Pose2D wantedPosition = new Pose2D(
                    DistanceUnit.INCH,
                    presetPositions[currentPreset].getX(DistanceUnit.INCH),
                    presetPositions[currentPreset].getY(DistanceUnit.INCH),
                    AngleUnit.DEGREES,
                    wantedHeading
            );
            driveTrain.setPositionDrive(wantedPosition, velocity);
        } else {
            // Start auto lock if y is pressed, and turn it off if driver turns
            if (gamepad1.y) {
                autoLock = true;
            }
            if (Math.abs(gamepad1.right_stick_x) > 0.001) {
                autoLock = false;
            }

            double turn = -gamepad1.right_stick_x * velocity;
            if (autoLock) {
                driveTrain.setWantedHeading(wantedHeading);
                turn = driveTrain.getHeadingCorrectionVelocity();
            }

            driveTrain.setVelocityDriveFieldCentric(-gamepad1.left_stick_y * velocity, -gamepad1.left_stick_x * velocity, turn);
        }

        telemetry.update();
        driveTrain.drive();
    }
}
