package org.firstinspires.ftc.teamcode.code.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.code.helpers.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.code.helpers.Prism.PrismAnimations;

@Config
@TeleOp(name = "LED Test", group = "TeleOp")
public class LEDTest extends OpMode {
    PrismAnimations.Solid solid = new PrismAnimations.Solid();
    PrismAnimations.Random random = new PrismAnimations.Random();
    PrismAnimations.PoliceLights policeLights = new PrismAnimations.PoliceLights();
    public static int red = 0;
    public static int green = 255;
    public static int blue = 0;
    public static float startHue = 0;
    public static float stopHue = 360;
    public static float speed = 0.05f;
    public static boolean doSolid = false;
    private GoBildaPrismDriver prism;
    private void prepareSolid(int red, int green, int blue) {
//        solid.setIndexes(0, 30);
        solid.setPrimaryColor(red, green, blue);
//        prism.insertAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, solid);
    }

    private void prepareRandom(float startHue, float stopHue, float speed) {
//        random.setIndexes(0, 30);
        random.setHues(startHue, stopHue);
        random.setSpeed(speed);
//        prism.insertAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, random);
    }

    @Override
    public void init() {
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        prepareRandom(startHue, stopHue, speed);
        prepareSolid(red, green, blue);
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, random);
            if (doSolid) {
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_2, solid);
            }
        }

        if (gamepad1.bWasPressed()) {
            prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, policeLights);
        }

        if (gamepad1.xWasPressed()) {
            prism.clearAllAnimations();
        }
    }
}
