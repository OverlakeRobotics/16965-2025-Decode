package org.firstinspires.ftc.teamcode.code.opmodes.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.code.helpers.AutoBase;

import java.io.FileReader;
import java.io.IOException;

@Config
@Autonomous(name = "Blue Autonomous Far", group = "Autonomous")
public class BlueAutonomousFar extends AutoBase {
    public static String jsonPath = "./jsonFiles/BlueAutonomousFar.json";

    @Override
    public void init() {
        super.readJson = true;
        super.init();
    }

    @Override
    public void start() {
        try {
            super.jsonReader = new FileReader(jsonPath);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        super.start();
    }
}
