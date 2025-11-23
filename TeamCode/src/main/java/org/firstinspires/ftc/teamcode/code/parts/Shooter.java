package org.firstinspires.ftc.teamcode.code.parts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class Shooter {
    public final double MAX_VELOCITY = 2800;
    public final double MOTOR_TO_WHEEL_VELOCITY_RATIO = 1.0;
    public static int minHoodAngle = 15;
    public static int maxHoodAngle = 50;
    private final DcMotorEx shooterMotor;
    private final Servo hoodServo;
    private final Servo blocker;
    public static double p = 500;
    public static double i = 1.5;
    public static double d = 5;
    public static double f = 0;

    public Shooter(DcMotorEx shooterMotor, Servo hoodServo, Servo blocker) {
        this.shooterMotor = shooterMotor;
//        this.shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.shooterMotor.setVelocityPIDFCoefficients(p, i, d, f);

        this.hoodServo = hoodServo;
        this.hoodServo.setDirection(Servo.Direction.REVERSE);
        this.blocker = blocker;
    }

    public void setShooterVelocity(double velocity) {
        shooterMotor.setVelocity(Range.clip(velocity * MOTOR_TO_WHEEL_VELOCITY_RATIO, -MAX_VELOCITY, MAX_VELOCITY));
    }

    public double getShooterVelocity() {
        return shooterMotor.getVelocity() / MOTOR_TO_WHEEL_VELOCITY_RATIO;
    }

    public double getShooterPower() {
        return shooterMotor.getPower() / MOTOR_TO_WHEEL_VELOCITY_RATIO;
    }

    // TODO: Make this set correct angle
    public void setHoodAngle(double angle) {
        hoodServo.setPosition(Range.clip(((Range.clip(angle, minHoodAngle, maxHoodAngle) - minHoodAngle) * 3) / 300, 0, 1));
    }

    public void open() {
        blocker.setPosition(0.2);
    }

    public void close() {
        blocker.setPosition(0.0);
    }
}