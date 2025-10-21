package org.firstinspires.ftc.teamcode.code.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.code.parts.Intake;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;

@Config
@TeleOp(name = "TeleOp Test", group = "TeleOp")
public class TeleOpTest extends OpMode {
    public double yOffset = -168.0; // mm
    public double xOffset = -84.0; // mm

    public double velocity = 2000;

    private OdometryHolonomicDrivetrain driveTrain;
    private Intake intake;
    private boolean intakeOn = false;
    private boolean intakeReversed = false;

    @Override
    public void init() {
        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpointDriver.setOffsets(xOffset, yOffset, DistanceUnit.MM);
        driveTrain = new OdometryHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                new GoBildaPinpointOdometry(pinpointDriver)
        );

        driveTrain.setCountsToSlowDown(500);

        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intakeMotor"));
    }

    @Override
    public void loop() {
        if (gamepad1.xWasPressed()) {
            intakeOn = !intakeOn;
        }
        if (gamepad1.aWasPressed()) {
            intakeReversed = false;
        }
        if (gamepad1.yWasPressed()) {
            intakeReversed = true;
        }
        if (intakeOn) {
            if (intakeReversed) {
                intake.setVelocity(-2000);
            } else {
                intake.setVelocity(2000);
            }
        } else {
            intake.setVelocity(0);
        }

        driveTrain.updatePosition();
        driveTrain.setVelocityDrive(
                -gamepad1.left_stick_y * velocity,
                -gamepad1.left_stick_x * velocity,
                -gamepad1.right_stick_x * velocity
        );
        driveTrain.drive();
    }
}
