package org.firstinspires.ftc.teamcode.code.opmodes;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.code.parts.Intake;

@Config
@TeleOp(name = "Intake Info", group = "TeleOp")
public class IntakeInfo extends OpMode {
    private Intake intake;
    @Override
    public void init() {
        this.intake = new Intake(
                hardwareMap.get(DcMotorEx.class, "intake")
//                hardwareMap.get(DistanceSensor.class, "distanceSensor")
        );
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            intake.setVelocity(2800);
        }
        if (gamepad1.bWasPressed()) {
            intake.setVelocity(0);
        }
//        Log.d("Intake Info", "Distance MM" + intake.getDistanceMM());
//        Log.d("Intake Info", "Velocity" + intake.getVelocity());
//        Log.d("Intake Info", "Power" + intake.getPower());
//        Log.d("Intake Info", "Current" + intake.getCurrent());
//        telemetry.addData("Distance MM", intake.getDistanceMM());
//        telemetry.addData("Velocity", intake.getVelocity());
//        telemetry.addData("Power", intake.getPower());
//        telemetry.addData("Current", intake.getCurrent());
//        telemetry.update();
    }
}
