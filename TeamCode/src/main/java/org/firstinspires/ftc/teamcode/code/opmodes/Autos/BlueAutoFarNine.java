package org.firstinspires.ftc.teamcode.code.opmodes.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.code.helpers.BaseAuto;

@Config
@Autonomous(name = "Blue Auto Far Nine", group = "Autonomous")
public class BlueAutoFarNine extends BaseAuto {
    @Override
    public void init() {
        super.readJson = true;
        super.jsonFilename = "pathJsons/NineArtifactFarBlue.json";
        super.init();
    }
}