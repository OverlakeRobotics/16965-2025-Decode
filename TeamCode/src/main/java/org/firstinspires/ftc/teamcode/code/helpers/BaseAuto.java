package org.firstinspires.ftc.teamcode.code.helpers;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.code.parts.Intake;
import org.firstinspires.ftc.teamcode.code.parts.Turret;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.Reader;
import java.util.Arrays;

import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.PathServer;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

@Config
public abstract class BaseAuto extends OpMode {
    public static final double yOffset = -156.0; // -168.0 // mm
    public static final double xOffset = 72.0; // -84.0 // mm

    protected int velocity;
    protected int intakeVelocity;
    protected double tolerance;
    protected Reader jsonReader;
    protected boolean readJson;
    protected String jsonFilename;
    protected OdometryHolonomicDrivetrain driveTrain;
    private Intake intake;
    private Turret turret;
    private AutoAligner autoAligner;
    private Limelight3A limelight;
    private final ElapsedTime runtime = new ElapsedTime();

    protected Pose2D[] positions;
    protected PathServer.Tag[] tags;

    public static double p = 30; // 19; // 30;
    public static double i = 0; // 0.1; // 0;
    public static double d = 7; // 0; // 7;
    public static double f = 0;

    private double wantedShooterVelocity = 0;
    private boolean isShooting;
    public static double shooterDelay = 0.3;
    private double shooterTimer = 0;
    public static double shooterTolerance = 80;
    public static double turretTolerance = 5;
    public String alliance;

    protected Pose2D startPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    private int lastTagIndex = 0;
    private double lastTime = 0;
    private double pauseTimeLeft = 0;
    private int pausedIndex = -1;
    private int autoAlignIndex = -1;

    // Gets the data for a path from a json, as a string.
    public void parseJsonFromString() throws IOException, JSONException {
        BufferedReader br = new BufferedReader(jsonReader);
        StringBuilder sb = new StringBuilder();
        String line;
        while ((line = br.readLine()) != null) sb.append(line);
        JSONObject root = new JSONObject(sb.toString());

        // Velocity (inches/sec) -> encoder counts/sec
        double velInches = root.optDouble("velocity", 0);
        velocity = (int) (velInches * BasicHolonomicDrivetrain.FORWARD_COUNTS_PER_INCH);

        // Tolerance (inches)
        tolerance = root.optDouble("tolerance", 1.0);

        alliance = root.optString("alliance", alliance);
        alliance = alliance.trim().toLowerCase();

        // Start pose (support object or array formats)
        double sx = 0;
        double sy = 0;
        double sh = 0;
        Object startObj = root.opt("start");
        if (startObj instanceof JSONArray) {
            JSONArray s = (JSONArray) startObj;
            sx = s.optDouble(0, 0);
            sy = s.optDouble(1, 0);
            sh = s.optDouble(2, 0);
        } else if (startObj instanceof JSONObject) {
            JSONObject s = (JSONObject) startObj;
            sx = s.optDouble("x", 0);
            sy = s.optDouble("y", 0);
            sh = s.optDouble("h", 0);
        }
        startPose = new Pose2D(DistanceUnit.INCH, sx, sy, AngleUnit.DEGREES, sh);

        // Waypoints -> positions array
        JSONArray pts = root.getJSONArray("points");
        positions = new Pose2D[pts.length()];
        for (int i = 0; i < pts.length(); i++) {
            Object pObj = pts.get(i);
            double x = 0;
            double y = 0;
            double h = 0;
            if (pObj instanceof JSONArray) {
                JSONArray p = (JSONArray) pObj;
                x = p.optDouble(0, 0);
                y = p.optDouble(1, 0);
                h = p.optDouble(2, 0);
            } else if (pObj instanceof JSONObject) {
                JSONObject p = (JSONObject) pObj;
                x = p.optDouble("x", 0);
                y = p.optDouble("y", 0);
                h = p.optDouble("h", 0);
            }
            positions[i] = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, h);
        }

        // Tags (optional)
        JSONArray jtags = root.optJSONArray("tags");
        if (jtags != null) {
            tags = new PathServer.Tag[jtags.length()];
            for (int i = 0; i < jtags.length(); i++) {
                JSONObject t = jtags.getJSONObject(i);
                PathServer.Tag tag = new PathServer.Tag(
                        t.getString("name"),
                        t.optDouble("value", 0.0),
                        t.getInt("index")
                );
                tags[i] = tag;
            }
            Arrays.sort(tags);
        } else {
            tags = new PathServer.Tag[0];
        }
        br.close();
    }

    @Override
    public void init() {
        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpointDriver.setOffsets(xOffset, yOffset, DistanceUnit.MM);
        driveTrain = new OdometryHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                new GoBildaPinpointOdometry(pinpointDriver)
        );

        driveTrain.setVelocityPIDFCoefficients(p, i, d, f);

        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intake"));
        turret = new Turret(
                hardwareMap.get(DcMotorEx.class, "shooterTop"),
                hardwareMap.get(DcMotorEx.class, "shooterBottom"),
                hardwareMap.get(DcMotorEx.class, "turret"),
                hardwareMap.get(Servo.class, "hood"),
                hardwareMap.get(Servo.class, "blocker")
        );
        turret.resetTurretEncoder();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        autoAligner = new AutoAligner(driveTrain, turret, limelight, false);
    }

    @Override
    public void init_loop() {
        driveTrain.updatePosition();
    }

    @Override
    public void start() {
        if (readJson) {
            try {
                // If a filename is specified, read from assets file
                if (jsonFilename != null && !jsonFilename.isEmpty()) {
                    InputStream is = hardwareMap.appContext.getAssets().open(jsonFilename);
                    jsonReader = new InputStreamReader(is);
                }
                parseJsonFromString();
            } catch (IOException | JSONException e) {
                throw new RuntimeException(e);
            }
        }
        driveTrain.setTolerance(tolerance);
        driveTrain.setPosition(startPose);
        driveTrain.setVelocity(velocity);
        Arrays.sort(tags);
        driveTrain.setPositionDrive(positions);
        turret.close();
    }

    public void autoAim(int pointIndex) {
        Pose2D curTarget = positions[pointIndex];
        positions[pointIndex] = new Pose2D(DistanceUnit.INCH, curTarget.getX(DistanceUnit.INCH),
                curTarget.getY(DistanceUnit.INCH), AngleUnit.DEGREES, autoAligner.getDrivetrainAutoAlignAngle());
        double hoodAngle = autoAligner.getOptimalHoodAngle();
        turret.setHoodAngle(hoodAngle);
        wantedShooterVelocity = autoAligner.getShooterVelocityFromAngle(hoodAngle);
        turret.setShooterVelocity(wantedShooterVelocity);
    }

    public void shoot(int pointIndex) {
        turret.open();
        autoAim(pointIndex);
        double shootingIntakeVelocity = 0;

        if (shooterTimer <= 0 && Math.abs(turret.getShooterVelocity() - wantedShooterVelocity) <= shooterTolerance && Math.abs(turret.getTurretTargetAngle() - turret.getTurretCurrentAngle()) <= turretTolerance) {
            shootingIntakeVelocity = 2800;
        }

        intake.setVelocity(shootingIntakeVelocity);
    }

    @Override
    public void loop() {
        driveTrain.updatePosition();
        Pose2D pos = driveTrain.getPosition();
        Log.d("Pinpoint pos", pos.toString());
        Log.d("Shooter Velocity", "Turret Error: " + (turret.getTurretTargetAngle() - turret.getTurretCurrentAngle()));

        autoAligner.updateInterpolation(pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH));
        turret.setTurretAngle(autoAligner.getTurretAutoAlignAngle());

        double curTime = runtime.seconds();
        double dt = curTime - lastTime;
        lastTime = curTime;

        if (pauseTimeLeft <= 0) {
            driveTrain.drive();
            int nextPointIndex = driveTrain.getNextPointIndex();

            if (nextPointIndex == autoAlignIndex && nextPointIndex != -1) {
                autoAim(nextPointIndex);
            } else {
                autoAlignIndex = -1;
            }

            if (isShooting) {
                shoot(nextPointIndex);
            }

            while (lastTagIndex < tags.length && tags[lastTagIndex].index <= nextPointIndex) {
                PathServer.Tag currTag = tags[lastTagIndex];
                switch (currTag.name) {
                    case "velocity":
                        velocity = (int) (currTag.value * BasicHolonomicDrivetrain.FORWARD_COUNTS_PER_INCH);
                        driveTrain.setVelocity(velocity);
                        break;
                    case "pause":
                        if (currTag.value <= 0) break;
                        pauseTimeLeft = currTag.value;
                        Log.d("Pause Time Left", "Pause Time Left: " + pauseTimeLeft);
                        pausedIndex = nextPointIndex;
                        driveTrain.setPositionDrive(positions[nextPointIndex - 1]);
                        break;
                    case "intake":
                        if (currTag.value <= 0) {
                            intakeVelocity = 0;
                            intake.stop();
                        } else {
                            intakeVelocity = (int) Math.round(currTag.value);
                            intake.setVelocity(intakeVelocity);
                        }
                        break;
                    case "autoAim": {
                        if (alliance.equals("red")) {
                            autoAligner.setRed();
                        } else {
                            autoAligner.setBlue();
                        }
                        autoAlignIndex = nextPointIndex;
                        Pose2D curTarget2 = positions[nextPointIndex];
                        positions[nextPointIndex] = new Pose2D(DistanceUnit.INCH, curTarget2.getX(DistanceUnit.INCH), curTarget2.getY(DistanceUnit.INCH), AngleUnit.DEGREES, autoAligner.getDrivetrainAutoAlignAngle());
                        break;
                    }
                    case "shooterVelocity": {
                        turret.setShooterVelocity(currTag.value);
                        break;
                    }
                    case "hoodAngle": {
                        turret.setHoodAngle(currTag.value);
                        break;
                    }
                    case "launchArtifacts": {
                        if (currTag.value <= 0) break;
                        turret.open();
                        pauseTimeLeft = currTag.value;
                        Log.d("Pause Time Left", "Pause Time Left: " + pauseTimeLeft);
                        pausedIndex = nextPointIndex;
                        driveTrain.setPositionDrive(positions[nextPointIndex - 1]);
                        shooterTimer = shooterDelay;
                        isShooting = true;
                        break;
                    }
                    case "startLaunch": {
                        turret.open();
                        isShooting = true;
                        break;
                    }
                    case "endLaunch": {
                        turret.close();
                        intake.setVelocity(intakeVelocity);
                        isShooting = false;
                        break;
                    }
                }
                lastTagIndex++;
            }
        } else {
            if (isShooting) {
                shoot(pausedIndex - 1);
            }
            pauseTimeLeft -= dt;
            shooterTimer -= dt;

            Log.d("Pause Time Left", "Pause Time Left: " + pauseTimeLeft);

            if (pauseTimeLeft <= 0) {
                pauseTimeLeft = 0;
                driveTrain.setPositionDrive(positions, pausedIndex);
                if (isShooting) {
                    turret.close();
                    intake.setVelocity(intakeVelocity);
                    isShooting = false;
                }
            } else {
                driveTrain.setPositionDrive(positions[pausedIndex - 1]);
            }
        }
    }

    @Override
    public void stop() {

    }
}
