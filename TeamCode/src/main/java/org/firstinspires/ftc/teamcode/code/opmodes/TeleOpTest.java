package org.firstinspires.ftc.teamcode.code.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.code.parts.Intake;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;

@Config
@TeleOp(name = "TeleOp Test", group = "TeleOp")
@Disabled
public class TeleOpTest extends OpMode {
    public double yOffset = -168.0; // mm
    public double xOffset = -84.0; // mm

    public double velocity = 2000;

    private OdometryHolonomicDrivetrain driveTrain;
    private Intake intake;
    private boolean intakeOn = false;
    private boolean intakeReversed = false;
    private boolean intakeSlow = false;
    private int intakeNormalVelocity = 2000;
    private int intakeSlowVelocity = 1000;

    private DcMotorEx shooterMotor;
    private Servo hoodServo;
    private Servo shooterBlocker;
    private int shooterVelocity;
    private double hoodPos;

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

        intake = new Intake(
                hardwareMap.get(DcMotorEx.class, "intake")
//                hardwareMap.get(DistanceSensor.class, "distanceSensor")
        );
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        hoodServo = hardwareMap.get(Servo.class, "hood");
        shooterBlocker = hardwareMap.get(Servo.class, "blocker");
        shooterBlocker.setDirection(Servo.Direction.REVERSE);
        shooterVelocity = 0;
        hoodPos = 0.5;
        shooterBlocker.setPosition(0.0);
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
                if (intakeSlow) {
                    intake.setVelocity(-intakeSlowVelocity);
                } else {
                    intake.setVelocity(-intakeNormalVelocity);
                }
            } else {
                if (intakeSlow) {
                    intake.setVelocity(intakeSlowVelocity);
                } else {
                    intake.setVelocity(intakeNormalVelocity);
                }
            }
        } else {
            intake.setVelocity(0);
        }

        // Shooter controls
        if (gamepad1.rightBumperWasPressed()) {
            shooterVelocity += 100;
        } else if (gamepad1.leftBumperWasPressed()) {
            shooterVelocity -= 100;
        } else if (gamepad1.bWasPressed()) {
            shooterVelocity = 0;
        }

        shooterMotor.setVelocity(shooterVelocity);

        // Hood servo controls
        if (gamepad1.dpadUpWasPressed()) {
            hoodPos -= 0.02;
        } else if (gamepad1.dpadDownWasPressed()) {
            hoodPos += 0.02;
        }

        hoodPos = Range.clip(hoodPos, 0, 1);
        hoodServo.setPosition(hoodPos);

        // Shooter Block controls
        if (gamepad1.dpadLeftWasPressed()) {
            shooterBlocker.setPosition(0.15); // Open
            intakeSlow = true;
        } else if (gamepad1.dpadLeftWasReleased()) {
            shooterBlocker.setPosition(0.0); // Closed
            intakeSlow = false;
        }

        // Telemetry for shooter
        telemetry.addData("Wanted Shooter Velocity", shooterVelocity);
        telemetry.addData("Actual Shooter Velocity", shooterMotor.getVelocity());
        telemetry.addData("Shooter Power", shooterMotor.getPower());
        telemetry.addData("Hood Servo Position", hoodPos);

        driveTrain.updatePosition();
        driveTrain.setVelocityDrive(
                -gamepad1.left_stick_y * velocity,
                -gamepad1.left_stick_x * velocity,
                -gamepad1.right_stick_x * velocity
        );
        driveTrain.drive();
    }
}
