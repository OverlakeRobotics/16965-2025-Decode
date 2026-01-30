package org.firstinspires.ftc.teamcode.code.parts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Intake {
    public enum IntakeState {
        EMPTY,
        ONE,
        TWO,
        FULL,
        JAMMED,
        AMBIENT,
        OFF
    }
    private static final double LOWER_DISTANCE_THRESHOLD_MM = 200;
    private static final double MIDDLE_DISTANCE_THRESHOLD_MM = 50;
    private static final double UPPER_DISTANCE_THRESHOLD_MM = 100;
    private static final double MAX_VELOCITY = 2800;
    private final DcMotorEx intakeMotor;
    private final DistanceSensor lowerDistanceSensor;
    private final NormalizedColorSensor middleColorSensor;
    private final NormalizedColorSensor upperColorSensor;
    private final ElapsedTime runtime = new ElapsedTime();
//    private double blockedBeginTime = -1;
    private double stallBeginTime = -1;
    private double wantedVelocity = 0;

    public Intake(DcMotorEx intakeMotor, DistanceSensor lowerDistanceSensor, NormalizedColorSensor middleColorSensor, NormalizedColorSensor upperColorSensor) {
        this.intakeMotor = intakeMotor;
        this.intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.lowerDistanceSensor = lowerDistanceSensor;
        this.middleColorSensor = middleColorSensor;
        this.upperColorSensor = upperColorSensor;
    }

    public void setVelocity(double velocity) {
        wantedVelocity = Range.clip(velocity, -MAX_VELOCITY, MAX_VELOCITY);
        intakeMotor.setVelocity(wantedVelocity);
    }

    public double getVelocity() {
        return intakeMotor.getVelocity();
    }

    public double getPower() {
        return intakeMotor.getPower();
    }

    public double getCurrent() {
        return intakeMotor.getCurrent(CurrentUnit.AMPS);
    }

    public boolean isStalling() {
        boolean stalling = Math.abs(intakeMotor.getVelocity()) < Math.abs(wantedVelocity) * 0.2 && Math.abs(getCurrent()) > 3.0;
        if (!stalling) {
            stallBeginTime = -1;
            return false;
        } else if (stallBeginTime < 0) {
            stallBeginTime = runtime.seconds();
        }
        return runtime.seconds() - stallBeginTime > 0.5;
    }

    public IntakeState getState() {
        if (isStalling()) {
            return IntakeState.JAMMED;
        }
        int lowerBlocked = getLowerDistanceMM() < LOWER_DISTANCE_THRESHOLD_MM ? 1 : 0;
        int middleBlocked = getMiddleDistanceMM() < MIDDLE_DISTANCE_THRESHOLD_MM ? 1 : 0;
        int upperBlocked = getUpperDistanceMM() < UPPER_DISTANCE_THRESHOLD_MM ? 1 : 0;
        int totalBlocked = lowerBlocked + middleBlocked + upperBlocked;
        switch(totalBlocked) {
            case 1:
                return IntakeState.ONE;
            case 2:
                return IntakeState.TWO;
            case 3:
                return IntakeState.FULL;
            default:
                return IntakeState.EMPTY;
        }
//        Log.d("LED Latency", "Dist: " + distanceSensor.getDistance(DistanceUnit.MM));
//        if (!distanceSensorBlocked) {
//            blockedBeginTime = -1;
//            return IntakeState.EMPTY;
//        } else if (blockedBeginTime < 0) {
//            blockedBeginTime = runtime.seconds();
//        }
//        return runtime.seconds() - blockedBeginTime > 0.3 ? IntakeState.FULL : IntakeState.PASSING;
    }

    public double getLowerDistanceMM() {
        return lowerDistanceSensor.getDistance(DistanceUnit.MM);
    }

    public double getMiddleDistanceMM() {
        if (middleColorSensor instanceof DistanceSensor) {
            return ((DistanceSensor) middleColorSensor).getDistance(DistanceUnit.MM);
        } else {
            return -1;
        }
    }

    public double getUpperDistanceMM() {
        if (upperColorSensor instanceof DistanceSensor) {
            return ((DistanceSensor) upperColorSensor).getDistance(DistanceUnit.MM);
        } else {
            return -1;
        }
    }

    public void stop() {
        wantedVelocity = 0;
        this.setVelocity(wantedVelocity);
    }
}