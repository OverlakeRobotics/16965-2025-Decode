package org.firstinspires.ftc.teamcode.code.parts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@Config
public class Intake {
    private static final double MAX_VELOCITY = 2880;
    private final DcMotorEx intakeMotor;

    public Intake(DcMotorEx intakeMotor) {
        this.intakeMotor = intakeMotor;
        this.intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void setVelocity(double velocity) {
        intakeMotor.setVelocity(Range.clip(velocity, -MAX_VELOCITY, MAX_VELOCITY));
    }

    public double getVelocity() {
        return intakeMotor.getVelocity();
    }

    public void stop() {
        this.setVelocity(0);
    }
}