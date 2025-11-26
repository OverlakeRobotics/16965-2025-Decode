package org.firstinspires.ftc.teamcode.code.opmodes.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.code.helpers.BaseAuto;

@Config
@Autonomous(name = "Blue Auto Close Twelve", group = "Autonomous")
public class BlueAutoCloseTwelve extends BaseAuto {
    @Override
    public void init() {
        super.readJson = true;
        super.jsonFilename = "pathJsons/TwelveArtifactCloseBlue.json";
        super.init();
    }
}