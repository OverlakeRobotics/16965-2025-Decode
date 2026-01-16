package org.firstinspires.ftc.teamcode.code.parts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Config
public class Potentiometer {
    public static double DEGREES_PER_VOLT = 3600 / 3.33;
    public static double POTENTIOMETER_VOLTAGE_AT_TURRET_ZERO = 0.881;
    private final AnalogInput potentiometer;

    public Potentiometer(AnalogInput potentiometer) {
        this.potentiometer = potentiometer;
    }

    public double getAngleDegrees() {
        return (getVoltage() - POTENTIOMETER_VOLTAGE_AT_TURRET_ZERO) * DEGREES_PER_VOLT / 4;
    }

    public double getVoltage() {
        return potentiometer.getVoltage();
    }
}
