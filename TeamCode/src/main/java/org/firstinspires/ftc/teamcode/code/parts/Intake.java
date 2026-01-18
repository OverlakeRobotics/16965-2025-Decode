package org.firstinspires.ftc.teamcode.code.parts;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Intake {
    public enum IntakeState {
        AMBIENT,
        EMPTY,
        PASSING,
        FULL,
        JAMMED
    }
    private static final double DISTANCE_THRESHOLD_MM = 200;
    private static final double MAX_VELOCITY = 2800;
    private final DcMotorEx intakeMotor;
//    private final DistanceSensor distanceSensor;
//    private final ElapsedTime runtime = new ElapsedTime();
//    private double blockedBeginTime = -1;
//    private double stallBeginTime = -1;
    private double wantedVelocity = 0;

    public Intake(DcMotorEx intakeMotor/*, DistanceSensor distanceSensor*/) {
        this.intakeMotor = intakeMotor;
        this.intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        this.distanceSensor = distanceSensor;
    }

    public void setVelocity(double velocity) {
        wantedVelocity = Range.clip(velocity, -MAX_VELOCITY, MAX_VELOCITY);
        intakeMotor.setVelocity(wantedVelocity);
    }

    public double getVelocity() {
        return intakeMotor.getVelocity();
    }

//    public double getPower() {
//        return intakeMotor.getPower();
//    }
//
//    public double getCurrent() {
//        return intakeMotor.getCurrent(CurrentUnit.AMPS);
//    }
//
//    public boolean isStalling() {
//        boolean stalling = Math.abs(intakeMotor.getVelocity()) < wantedVelocity * 0.3;
//        if (!stalling) {
//            stallBeginTime = -1;
//            return false;
//        } else if (stallBeginTime < 0) {
//            stallBeginTime = runtime.seconds();
//        }
//        return runtime.seconds() - stallBeginTime > 0.25;
//    }
//
//    public IntakeState getState() {
//        if (isStalling()) {
//            return IntakeState.JAMMED;
//        }
//        boolean distanceSensorBlocked = distanceSensor.getDistance(DistanceUnit.MM) < DISTANCE_THRESHOLD_MM;
//        Log.d("LED Latency", "Dist: " + distanceSensor.getDistance(DistanceUnit.MM));
//        if (!distanceSensorBlocked) {
//            blockedBeginTime = -1;
//            return IntakeState.EMPTY;
//        } else if (blockedBeginTime < 0) {
//            blockedBeginTime = runtime.seconds();
//        }
//        return runtime.seconds() - blockedBeginTime > 0.5 ? IntakeState.FULL : IntakeState.PASSING;
//    }
//
//    public double getDistanceMM() {
//        return distanceSensor.getDistance(DistanceUnit.MM);
//    }

    public void stop() {
        wantedVelocity = 0;
        this.setVelocity(wantedVelocity);
    }
}