package org.firstinspires.ftc.teamcode.code.opmodes.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.code.helpers.BaseAuto;

@Config
@Autonomous(name = "Red Auto Close Twelve", group = "Autonomous")
public class RedAutoCloseTwelve extends BaseAuto {
    @Override
    public void init() {
        super.readJson = true;
        super.jsonFilename = "pathJsons/TwelveArtifactCloseRed.json";
        super.init();
    }
}