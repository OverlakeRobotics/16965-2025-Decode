package org.firstinspires.ftc.teamcode.code.opmodes.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.code.helpers.BaseAuto;

@Config
@Autonomous(name = "Red Auto Far Six", group = "Autonomous")
public class RedAutoFarSix extends BaseAuto {
    @Override
    public void init() {
        super.readJson = true;
        super.jsonFilename = "pathJsons/SixArtifactFarRed.json";
        super.init();
    }
}
