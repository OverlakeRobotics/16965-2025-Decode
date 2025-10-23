package org.firstinspires.ftc.teamcode.code.parts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class Shooter {
    private static final double MAX_VELOCITY = 2800;
    private final DcMotorEx shooterMotor;
    private final Servo hoodServo;
    private final Servo blocker;

    public Shooter(DcMotorEx shooterMotor, Servo hoodServo, Servo blocker) {
        this.shooterMotor = shooterMotor;
        this.shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        this.hoodServo = hoodServo;
        this.blocker = blocker;
    }

    public void setVelocity(double velocity) {
        shooterMotor.setVelocity(Range.clip(velocity, -MAX_VELOCITY, MAX_VELOCITY));
    }

    public double getVelocity() {
        return shooterMotor.getVelocity();
    }

    // TODO: Make this set correct angle
    public void setAngle(double angle) {
        hoodServo.setPosition(Range.clip(angle / 100, 0, 1));
    }

    public void open() {
        blocker.setPosition(0.15);
    }

    public void close() {
        blocker.setPosition(0.0);
    }
}