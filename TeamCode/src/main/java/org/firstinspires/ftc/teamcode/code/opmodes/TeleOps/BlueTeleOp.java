package org.firstinspires.ftc.teamcode.code.opmodes.TeleOps;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.code.helpers.BaseTeleOp;

@Config
@TeleOp(name = "Blue TeleOp", group = "TeleOp")
public class BlueTeleOp extends BaseTeleOp {
    @Override
    public Pose2D[] getPresetPositions() {
        return new Pose2D[]{
                new Pose2D(DistanceUnit.INCH, -54, 15, AngleUnit.DEGREES, 0),
                new Pose2D(DistanceUnit.INCH, 15, 15, AngleUnit.DEGREES, 0),
        };
    }

    @Override
    protected boolean isRedAlliance() {
        return false;
    }
}