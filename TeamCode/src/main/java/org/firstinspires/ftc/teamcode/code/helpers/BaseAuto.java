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
    public static double shooterTolerance = 60;

    protected Pose2D startPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    private int lastTagIndex = 0;
    private double lastTime = 0;
    private double pauseTimeLeft = 0;
    private int pausedIndex = 0;
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

        // Start pose
        JSONObject s = root.getJSONObject("start");
        double sx = s.getDouble("x");
        double sy = s.getDouble("y");
        double sh = s.optDouble("h", 0);
        startPose = new Pose2D(DistanceUnit.INCH, sx, sy, AngleUnit.DEGREES, sh);

        // Waypoints -> positions array
        JSONArray pts = root.getJSONArray("points");
        positions = new Pose2D[pts.length()];
        for (int i = 0; i < pts.length(); i++) {
            JSONObject p = pts.getJSONObject(i);
            double x = p.getDouble("x");
            double y = p.getDouble("y");
            double h = p.optDouble("h", 0);
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
                new GoBildaPinpointOdometry(pinpointDriver),
                p, i, d, f
        );

        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intake"));
        turret = new Turret(hardwareMap.get(DcMotorEx.class, "shooter"),
                hardwareMap.get(DcMotorEx.class, "turret"),
                hardwareMap.get(Servo.class, "hood"),
                hardwareMap.get(Servo.class, "blocker"));
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
        Arrays.sort(tags);
        driveTrain.setPositionDrive(positions, velocity);
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

    @Override
    public void loop() {
        driveTrain.updatePosition();
        Pose2D pos = driveTrain.getPosition();
        autoAligner.updateInterpolation(pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH));
        turret.setTurretAngle(autoAligner.getTurretAutoAlignAngle());
//        turret.setTurretAngle(0);

        if (pauseTimeLeft <= 0) {
            driveTrain.drive();
            int nextPointIndex = driveTrain.getNextPointIndex();

            if (nextPointIndex == autoAlignIndex && nextPointIndex != -1) {
                autoAim(nextPointIndex);
//                autoAim();
            } else {
                autoAlignIndex = -1;
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
                        pausedIndex = nextPointIndex;
                        driveTrain.setPositionDrive(positions[nextPointIndex - 1], velocity);
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
                    case "autoAimRed": {
                        autoAlignIndex = nextPointIndex;
                        autoAligner.setRed();
                        Pose2D curTarget2 = positions[nextPointIndex];
                        positions[nextPointIndex] = new Pose2D(DistanceUnit.INCH, curTarget2.getX(DistanceUnit.INCH), curTarget2.getY(DistanceUnit.INCH), AngleUnit.DEGREES, autoAligner.getDrivetrainAutoAlignAngle());
                        break;
                    }
                    case "autoAimBlue": {
                        autoAlignIndex = nextPointIndex;
                        autoAligner.setBlue();
                        Pose2D curTarget3 = positions[nextPointIndex];
                        positions[nextPointIndex] = new Pose2D(DistanceUnit.INCH, curTarget3.getX(DistanceUnit.INCH), curTarget3.getY(DistanceUnit.INCH), AngleUnit.DEGREES, autoAligner.getDrivetrainAutoAlignAngle());
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
                        pausedIndex = nextPointIndex;
                        driveTrain.setPositionDrive(positions[nextPointIndex - 1], velocity);
                        shooterTimer = shooterDelay;
                        isShooting = true;
                        break;
                    }
                }
                lastTagIndex++;
            }
        } else {
            if (isShooting) {
                turret.open();
                autoAim(pausedIndex - 1);
//                autoAim();
                double shootingIntakeVelocity = 0;

                if (shooterTimer <= 0 && Math.abs(turret.getShooterVelocity() - wantedShooterVelocity) <= shooterTolerance) {
                    shootingIntakeVelocity = 2000;
                }

                intake.setVelocity(shootingIntakeVelocity);
            }
            double dt = runtime.seconds() - lastTime;
            pauseTimeLeft -= dt;
            shooterTimer -= dt;

            if (pauseTimeLeft <= 0) {
                pauseTimeLeft = 0;
                // Initial point index is wrong or something.
                driveTrain.setPositionDrive(positions, velocity, pausedIndex);
                if (isShooting) {
                    turret.close();
                    intake.setVelocity(intakeVelocity);
                    isShooting = false;
                }
            } else {
                driveTrain.setPositionDrive(positions[pausedIndex - 1], velocity);
            }
        }
        Log.d("Loop Time", Double.toString(runtime.seconds() - lastTime));
        lastTime = runtime.seconds();
    }

    @Override
    public void stop() {

    }
}
