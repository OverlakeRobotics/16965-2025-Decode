package org.firstinspires.ftc.teamcode.code.parts;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Turret extends Shooter {
    public final double TICKS_PER_DEGREE = 537.7 / 360;
    public final int ANGLE_LIMIT = 90; // degrees
    private final int TURRET_VELOCITY = 2800;
    private final DcMotorEx turretMotor;
    public Turret(DcMotorEx shooterMotor, DcMotorEx turretMotor, Servo hoodServo, Servo blocker) {
        super(shooterMotor, hoodServo, blocker);
        this.turretMotor = turretMotor;
        this.turretMotor.setTargetPosition(0);
        this.turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        this.turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    // Left is positive, right is negative
    public void setTurretAngle(double angle) {
        turretMotor.setTargetPosition((int) Math.round(Range.clip(angle, -ANGLE_LIMIT, ANGLE_LIMIT) * 4 * TICKS_PER_DEGREE));
        turretMotor.setVelocity(TURRET_VELOCITY);
    }

    public double getTurretTargetAngle() {
        return turretMotor.getTargetPosition() / (TICKS_PER_DEGREE * 4);
    }

    public double getTurretCurrentAngle() {
        return turretMotor.getCurrentPosition() / (TICKS_PER_DEGREE * 4);
    }

    public void resetTurretEncoder() {
        this.turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
}
