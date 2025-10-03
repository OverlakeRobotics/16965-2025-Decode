package org.firstinspires.ftc.teamcode.code.parts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class Intake {
    public enum IntakeState {
        RUNNING,
        STOPPED
    }
    private static final double MAX_VELOCITY = 2888;
    private double currentVelocity = 0;
    private final DcMotorEx intakeMotor;
    private IntakeState currentState = IntakeState.STOPPED;

    public Intake(DcMotorEx intakeMotor) {
        this.intakeMotor = intakeMotor;
        this.intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    private void setState(IntakeState newState) {
        switch (newState) {
            case RUNNING:
                intakeMotor.setVelocity(currentVelocity);
                break;
            case STOPPED:
                intakeMotor.setVelocity(0);
                break;
        }
        currentState = newState;
    }

    public void setVelocity(double velocity) {
        if (velocity == 0) {
            stop();
        } else {
            currentVelocity = Math.min(velocity, MAX_VELOCITY);
            setState(IntakeState.RUNNING);
        }
    }

    public void stop() {
        currentVelocity = 0;
        setState(IntakeState.STOPPED);
    }
}