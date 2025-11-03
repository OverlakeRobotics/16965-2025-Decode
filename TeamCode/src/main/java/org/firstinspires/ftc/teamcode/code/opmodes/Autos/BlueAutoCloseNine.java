package org.firstinspires.ftc.teamcode.code.opmodes.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.code.helpers.BaseAuto;

@Config
@Autonomous(name = "Blue Auto Close Nine", group = "Autonomous")
public class BlueAutoCloseNine extends BaseAuto {
    @Override
    public void init() {
        super.readJson = true;
        super.jsonFilename = "pathJsons/NineArtifactCloseBlue.json";
        super.init();
    }
}
