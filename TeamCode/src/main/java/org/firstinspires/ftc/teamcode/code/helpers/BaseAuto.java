package org.firstinspires.ftc.teamcode.code.helpers;

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
import org.firstinspires.ftc.teamcode.code.parts.Shooter;
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
    public double yOffset = -168.0; // mm
    public double xOffset = -84.0; // mm

    protected int velocity;
    protected double tolerance;
    protected Reader jsonReader;
    protected boolean readJson;
    protected String jsonFilename;
    protected OdometryHolonomicDrivetrain driveTrain;
    private Intake intake;
    private Shooter shooter;
    private AutoAligner autoAligner;
    private Limelight3A limelight;
    private final ElapsedTime runtime = new ElapsedTime();

    protected Pose2D[] positions;
    protected PathServer.Tag[] tags;

    private double wantedShooterVelocity = 0;
    private boolean isShooting;
    public static double shooterDelay = 0.6;
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
                        t.optInt("value", 0),
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

        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intake"));
        shooter = new Shooter(hardwareMap.get(DcMotorEx.class, "shooter"),
                hardwareMap.get(Servo.class, "hood"),
                hardwareMap.get(Servo.class, "blocker"));
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        autoAligner = new AutoAligner(driveTrain, limelight, false);
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
        shooter.close();
    }

    public void autoAim(int pointIndex) {
        Pose2D curTarget = positions[pointIndex];
        positions[pointIndex] = new Pose2D(DistanceUnit.INCH, curTarget.getX(DistanceUnit.INCH),
                curTarget.getY(DistanceUnit.INCH), AngleUnit.DEGREES, autoAligner.getAutoAlignAngle());
        double hoodAngle = autoAligner.getOptimalHoodAngle();
        shooter.setAngle(hoodAngle);
        wantedShooterVelocity = autoAligner.getShooterVelocityFromAngle(hoodAngle);
        shooter.setVelocity(wantedShooterVelocity);
    }

    @Override
    public void loop() {
        driveTrain.updatePosition();

        if (pauseTimeLeft <= 0) {
            driveTrain.drive();
            int nextPointIndex = driveTrain.getNextPointIndex();

            if (nextPointIndex == autoAlignIndex && nextPointIndex != -1) {
                autoAim(nextPointIndex);
            } else {
                autoAlignIndex = -1;
            }

            while (lastTagIndex < tags.length && tags[lastTagIndex].index <= nextPointIndex) {
                PathServer.Tag currTag = tags[lastTagIndex];
                switch (currTag.name) {
                    case "velocity":
                        driveTrain.setVelocity((int) (currTag.value * BasicHolonomicDrivetrain.FORWARD_COUNTS_PER_INCH));
                        break;
                    case "pause":
                        pauseTimeLeft += currTag.value;
                        pausedIndex = nextPointIndex;
                        driveTrain.setPositionDrive(positions[nextPointIndex - 1], velocity);
                        break;
                    case "intake":
                        if (currTag.value <= 0) {
                            intake.stop();
                        } else {
                            intake.setVelocity(currTag.value);
                        }
                        break;
                    case "autoAimRed": {
                        autoAlignIndex = nextPointIndex;
                        autoAligner.setRed();
                        Pose2D curTarget2 = positions[nextPointIndex];
                        positions[nextPointIndex] = new Pose2D(DistanceUnit.INCH, curTarget2.getX(DistanceUnit.INCH), curTarget2.getY(DistanceUnit.INCH), AngleUnit.DEGREES, autoAligner.getAutoAlignAngle());
                        break;
                    }
                    case "autoAimBlue": {
                        autoAlignIndex = nextPointIndex;
                        autoAligner.setBlue();
                        Pose2D curTarget3 = positions[nextPointIndex];
                        positions[nextPointIndex] = new Pose2D(DistanceUnit.INCH, curTarget3.getX(DistanceUnit.INCH), curTarget3.getY(DistanceUnit.INCH), AngleUnit.DEGREES, autoAligner.getAutoAlignAngle());
                        break;
                    }
                    case "shooterVelocity": {
                        shooter.setVelocity(currTag.value);
                        break;
                    }
                    case "hoodAngle": {
                        shooter.setAngle(currTag.value);
                        break;
                    }
                    case "launchArtifacts": {
                        shooter.open();
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
                shooter.open();
                autoAim(pausedIndex - 1);
                double intakeVelocity = 0;

                if (shooterTimer <= 0 && Math.abs(shooter.getVelocity() - wantedShooterVelocity) <= shooterTolerance) {
                    intakeVelocity = 2000;
                }

                intake.setVelocity(intakeVelocity);
            }
            double dt = runtime.seconds() - lastTime;
            pauseTimeLeft -= dt;
            shooterTimer -= dt;

            if (pauseTimeLeft <= 0) {
                pauseTimeLeft = 0;
                // Initial point index is wrong or something.
                driveTrain.setPositionDrive(positions, velocity, pausedIndex);
                if (isShooting) {
                    shooter.close();
                    intake.setVelocity(0);
                    isShooting = false;
                }
            } else {
                driveTrain.setPositionDrive(positions[pausedIndex - 1], velocity);
            }
        }
        lastTime = runtime.seconds();
    }

    @Override
    public void stop() {

    }
}
