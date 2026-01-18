package org.firstinspires.ftc.teamcode.code.parts;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.code.helpers.Prism.Color;
import org.firstinspires.ftc.teamcode.code.helpers.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.code.helpers.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.code.parts.Intake.IntakeState;

@Config
public class LEDIndicator {
    private final PrismAnimations.Solid empty = new PrismAnimations.Solid(Color.ORANGE);
    private final PrismAnimations.Solid passing = new PrismAnimations.Solid(Color.BLUE);
    private final PrismAnimations.Solid full = new PrismAnimations.Solid(Color.GREEN);
    private final PrismAnimations.Blink jammed = new PrismAnimations.Blink(Color.RED);
    private final PrismAnimations.Rainbow ambient = new PrismAnimations.Rainbow();
    private final GoBildaPrismDriver prism;
    private IntakeState currentState;

    public LEDIndicator(GoBildaPrismDriver prism) {
        this.prism = prism;
        this.currentState = null;
    }

    public void setState(IntakeState state) {
        if (state == currentState) {
            Log.d("LED Latency", "Skipped");
            return;
        }
        currentState = state;
        prism.clearAllAnimations();
        switch (state) {
            case AMBIENT:
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, ambient);
                break;
            case EMPTY:
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, empty);
                break;
            case PASSING:
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, passing);
                break;
            case FULL:
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, full);
                break;
            case JAMMED:
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, jammed);
                break;
        }
    }
}
