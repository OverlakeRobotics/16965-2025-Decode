package org.firstinspires.ftc.teamcode.code.opmodes.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.code.helpers.BaseAuto;

import java.io.StringReader;

@Config
@Autonomous(name = "Blue Auto Close Six", group = "Autonomous")
public class BlueAutoCloseSix extends BaseAuto {
    @Override
    public void init() {
        super.readJson = true;
        super.jsonFilename = "pathJsons/SixArtifactCloseBlue.json";
        super.init();
    }
}
