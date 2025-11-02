package org.firstinspires.ftc.teamcode.code.opmodes.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.code.helpers.BaseAuto;

import java.io.StringReader;

@Config
@Autonomous(name = "Red Auto Close Nine", group = "Autonomous")
public class RedAutoCloseNine extends BaseAuto {
    public static String data = "{\n" +
            "  \"version\": 1,\n" +
            "  \"createdAt\": \"2025-11-02T04:34:04.978Z\",\n" +
            "  \"start\": {\n" +
            "    \"x\": 48,\n" +
            "    \"y\": -48,\n" +
            "    \"h\": 0\n" +
            "  },\n" +
            "  \"points\": [\n" +
            "    {\n" +
            "      \"x\": 48,\n" +
            "      \"y\": -48,\n" +
            "      \"h\": 0\n" +
            "    },\n" +
            "    {\n" +
            "      \"x\": 18,\n" +
            "      \"y\": -18,\n" +
            "      \"h\": 0\n" +
            "    },\n" +
            "    {\n" +
            "      \"x\": 9,\n" +
            "      \"y\": -27,\n" +
            "      \"h\": -90\n" +
            "    },\n" +
            "    {\n" +
            "      \"x\": 9,\n" +
            "      \"y\": -54,\n" +
            "      \"h\": -90\n" +
            "    },\n" +
            "    {\n" +
            "      \"x\": 18,\n" +
            "      \"y\": -18,\n" +
            "      \"h\": 0\n" +
            "    },\n" +
            "    {\n" +
            "      \"x\": -15,\n" +
            "      \"y\": -24,\n" +
            "      \"h\": -90\n" +
            "    },\n" +
            "    {\n" +
            "      \"x\": -15,\n" +
            "      \"y\": -54,\n" +
            "      \"h\": -90\n" +
            "    },\n" +
            "    {\n" +
            "      \"x\": 18,\n" +
            "      \"y\": -18,\n" +
            "      \"h\": 0\n" +
            "    },\n" +
            "    {\n" +
            "      \"x\": 54,\n" +
            "      \"y\": -18,\n" +
            "      \"h\": 0\n" +
            "    }\n" +
            "  ],\n" +
            "  \"headingMode\": \"straight\",\n" +
            "  \"endHeading\": 0,\n" +
            "  \"velocity\": 70,\n" +
            "  \"maxAccel\": 40,\n" +
            "  \"tolerance\": 5,\n" +
            "  \"snapInches\": 3,\n" +
            "  \"robot\": {\n" +
            "    \"length\": 18,\n" +
            "    \"width\": 18\n" +
            "  },\n" +
            "  \"tags\": [\n" +
            "    {\n" +
            "      \"index\": 1,\n" +
            "      \"name\": \"autoAimRed\",\n" +
            "      \"value\": 0\n" +
            "    },\n" +
            "    {\n" +
            "      \"index\": 2,\n" +
            "      \"name\": \"launchArtifacts\",\n" +
            "      \"value\": 4\n" +
            "    },\n" +
            "    {\n" +
            "      \"index\": 3,\n" +
            "      \"name\": \"intake\",\n" +
            "      \"value\": 2000\n" +
            "    },\n" +
            "    {\n" +
            "      \"index\": 3,\n" +
            "      \"name\": \"velocity\",\n" +
            "      \"value\": 25\n" +
            "    },\n" +
            "    {\n" +
            "      \"index\": 4,\n" +
            "      \"name\": \"velocity\",\n" +
            "      \"value\": 70\n" +
            "    },\n" +
            "    {\n" +
            "      \"index\": 4,\n" +
            "      \"name\": \"intake\",\n" +
            "      \"value\": 0\n" +
            "    },\n" +
            "    {\n" +
            "      \"index\": 4,\n" +
            "      \"name\": \"autoAimRed\",\n" +
            "      \"value\": 0\n" +
            "    },\n" +
            "    {\n" +
            "      \"index\": 5,\n" +
            "      \"name\": \"launchArtifacts\",\n" +
            "      \"value\": 4\n" +
            "    },\n" +
            "    {\n" +
            "      \"index\": 6,\n" +
            "      \"name\": \"velocity\",\n" +
            "      \"value\": 25\n" +
            "    },\n" +
            "    {\n" +
            "      \"index\": 6,\n" +
            "      \"name\": \"intake\",\n" +
            "      \"value\": 2000\n" +
            "    },\n" +
            "    {\n" +
            "      \"index\": 7,\n" +
            "      \"name\": \"intake\",\n" +
            "      \"value\": 0\n" +
            "    },\n" +
            "    {\n" +
            "      \"index\": 7,\n" +
            "      \"name\": \"velocity\",\n" +
            "      \"value\": 70\n" +
            "    },\n" +
            "    {\n" +
            "      \"index\": 7,\n" +
            "      \"name\": \"autoAimRed\",\n" +
            "      \"value\": 0\n" +
            "    },\n" +
            "    {\n" +
            "      \"index\": 8,\n" +
            "      \"name\": \"launchArtifacts\",\n" +
            "      \"value\": 4\n" +
            "    }\n" +
            "  ]\n" +
            "}";

    @Override
    public void init() {
        super.readJson = true;
        super.init();
    }

    @Override
    public void start() {
        super.jsonReader = new StringReader(data);
        super.start();
    }
}
