package org.firstinspires.ftc.teamcode.code.parts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class Turret extends Shooter {
    public final double TICKS_PER_DEGREE = 537.7 / 360;
    public static int MAX_LIMIT_ANGLE = 135; // 115; // degrees
    public static int MIN_LIMIT_ANGLE = -135; // -115; // degrees
    public static double turretP = 10;
    public static double turretI = 1;
    public static double turretD = 2;
    public static double turretF = 0;
    public final int MAX_ANGLE_LIMIT;
    public final int MIN_ANGLE_LIMIT;
    public static int TURRET_VELOCITY = 2800;
    private final DcMotorEx turretMotor;
    public Turret(DcMotorEx shooterMotor1, DcMotorEx shooterMotor2, DcMotorEx turretMotor, Servo hoodServo, Servo blocker) {
        super(shooterMotor1, shooterMotor2, hoodServo, blocker);
        this.MAX_ANGLE_LIMIT = MAX_LIMIT_ANGLE;
        this.MIN_ANGLE_LIMIT = MIN_LIMIT_ANGLE;
        this.turretMotor = turretMotor;
        this.turretMotor.setTargetPosition(0);
        this.turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        this.turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        this.turretMotor.setVelocityPIDFCoefficients(turretP, turretI, turretD, turretF);
    }

    // CCW is positive, CW is negative
    public void setTurretAngle(double angle) {
        turretMotor.setTargetPosition((int) Math.round(Range.clip(angle, MIN_ANGLE_LIMIT, MAX_ANGLE_LIMIT) * 4 * TICKS_PER_DEGREE));
        turretMotor.setVelocity(TURRET_VELOCITY);
    }

    public double getTurretTargetAngle() {
        return turretMotor.getTargetPosition() / (TICKS_PER_DEGREE * 4);
    }

    public double getTurretCurrentAngle() {
        return turretMotor.getCurrentPosition() / (TICKS_PER_DEGREE * 4);
    }

    public void resetTurretEncoder() {
        turretMotor.setVelocity(0);
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
}
