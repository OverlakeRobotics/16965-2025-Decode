package org.firstinspires.ftc.teamcode.code.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.code.helpers.BaseTeleOp;

@Config
@TeleOp(name = "Blue TeleOp", group = "TeleOp")
public class BlueTeleOp extends BaseTeleOp {
    // Blue alliance starting position
    // Positive angle is to the left, positive x is forward, and positive y is left
    // This is the center of the bot when the program is initialized
    public static final Pose2D BLUE_START_POS = new Pose2D(DistanceUnit.INCH, -63, 15, AngleUnit.DEGREES, 0);

    @Override
    protected Pose2D getStartPosition() {
        return BLUE_START_POS;
    }

    @Override
    protected boolean isRedAlliance() {
        return false;
    }
}