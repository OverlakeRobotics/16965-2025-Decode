package org.firstinspires.ftc.teamcode.code.parts;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
//        this.shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        this.hoodServo = hoodServo;
        this.blocker = blocker;
        this.blocker.setDirection(Servo.Direction.REVERSE);
    }

    public void setVelocity(double velocity) {
        shooterMotor.setVelocity(Range.clip(velocity, -MAX_VELOCITY, MAX_VELOCITY));
    }

    public PIDFCoefficients getPIDFCoefficients() {
        return shooterMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void setPIDF(PIDFCoefficients pidf) {
        shooterMotor.setVelocityPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
    }

    public double getVelocity() {
        return shooterMotor.getVelocity();
    }

    public double getPower() {
        return shooterMotor.getPower();
    }

    // TODO: Make this set correct angle
    public void setAngle(double angle) {
        hoodServo.setPosition(Range.clip((90 - angle) / 100, 0, 1));
    }

    public void open() {
        blocker.setPosition(0.18);
    }

    public void close() {
        blocker.setPosition(0.0);
    }
}