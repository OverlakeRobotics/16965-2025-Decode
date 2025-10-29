package org.firstinspires.ftc.teamcode.code.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.code.helpers.AutoAligner;
import org.firstinspires.ftc.teamcode.code.parts.Intake;
import org.firstinspires.ftc.teamcode.code.parts.Shooter;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.PathServer;

import java.util.Arrays;
import java.util.List;

@Config
@Autonomous(name = "Path Planner", group = "Autonomous")
public class PathPlanner extends OpMode {
    public double yOffset = -168.0; // mm
    public double xOffset = -84.0; // mm

    public static int velocity;
    public static double tolerance;

    private OdometryHolonomicDrivetrain driveTrain;
    private Intake intake;
    private Shooter shooter;
    private AutoAligner autoAligner;
    private Limelight3A limelight;
    public Pose2D[] positions;
    public PathServer.Tag[] tags;
    private int lastTagIndex = 0;
    private final ElapsedTime runtime = new ElapsedTime();
    private double lastTime = 0;
    private double pauseTimeLeft = 0;
    public int addIndex = 0;
    private int autoAlignIndex = -1;
    private double wantedShooterVelocity = 0;

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

        PathServer.startServer();
    }

    @Override
    public void start() {
        velocity = (int) (PathServer.getVelocity() * BasicHolonomicDrivetrain.FORWARD_COUNTS_PER_INCH);
        tolerance = PathServer.getTolerance();
        positions = PathServer.getPath();
        driveTrain.setPosition(PathServer.getStartPose());
        tags = PathServer.getTags();
        Arrays.sort(tags);
        driveTrain.setPositionDrive(positions, velocity, tolerance);
        shooter.close();
    }

    @Override
    public void loop() {
        driveTrain.updatePosition();
        PathServer.setRobotPose(driveTrain.getPosition());
        if (pauseTimeLeft <= 0) {
            driveTrain.drive();
            int nextPointIndex = driveTrain.getNextPointIndex();

            if (nextPointIndex == autoAlignIndex && nextPointIndex != -1) {
                Pose2D curTarget = positions[nextPointIndex];
                positions[nextPointIndex] = new Pose2D(DistanceUnit.INCH, curTarget.getX(DistanceUnit.INCH),
                        curTarget.getY(DistanceUnit.INCH), AngleUnit.DEGREES, autoAligner.getAutoAlignAngle());
                double hoodAngle = autoAligner.getOptimalHoodAngle();
                shooter.setAngle(hoodAngle);
                wantedShooterVelocity = autoAligner.getShooterVelocityFromAngle(hoodAngle);
                shooter.setVelocity(wantedShooterVelocity);
            } else {
                autoAlignIndex = -1;
            }

            int pauseIndexIncrement = 0;

            while (lastTagIndex < tags.length && tags[lastTagIndex].index <= nextPointIndex + addIndex) {
                PathServer.Tag currTag = tags[lastTagIndex];
                switch (currTag.name) {
                    case "velocity":
                        driveTrain.setVelocity((int) (currTag.value * BasicHolonomicDrivetrain.FORWARD_COUNTS_PER_INCH));
                        break;
                    case "pause":
                        pauseTimeLeft += currTag.value;
                        positions = Arrays.copyOfRange(positions, nextPointIndex, positions.length);
                        pauseIndexIncrement = nextPointIndex;
                        driveTrain.stop();
                        break;
                    case "intake":
                        if (currTag.value <= 0) {
                            intake.stop();
                        } else {
                            intake.setVelocity(currTag.value);
                        }
                        break;
                    case "autoAlignRed": {
                        autoAlignIndex = nextPointIndex;
                        autoAligner.setRed();
                        Pose2D curTarget = positions[nextPointIndex];
                        positions[nextPointIndex] = new Pose2D(DistanceUnit.INCH, curTarget.getX(DistanceUnit.INCH), curTarget.getY(DistanceUnit.INCH), AngleUnit.DEGREES, autoAligner.getAutoAlignAngle());
                        break;
                    }
                    case "autoAlignBlue": {
                        autoAlignIndex = nextPointIndex;
                        autoAligner.setBlue();
                        Pose2D curTarget = positions[nextPointIndex];
                        positions[nextPointIndex] = new Pose2D(DistanceUnit.INCH, curTarget.getX(DistanceUnit.INCH), curTarget.getY(DistanceUnit.INCH), AngleUnit.DEGREES, autoAligner.getAutoAlignAngle());
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
                        double startTime = runtime.seconds();
                        double intakeVelocity;
                        while (runtime.seconds() - startTime < currTag.value) {
                            shooter.open();
                            intakeVelocity = 0;

                            if (Math.abs(shooter.getVelocity() - wantedShooterVelocity) <= 40) {
                                intakeVelocity = 2000;
                            }
                            intake.setVelocity(intakeVelocity);
                        }
                        intake.setVelocity(0);
                        shooter.close();
                    }
                }
                lastTagIndex++;
            }

            addIndex += pauseIndexIncrement;
        } else {
            pauseTimeLeft -= runtime.seconds() - lastTime;
            if (pauseTimeLeft <= 0) {
                pauseTimeLeft = 0;
                driveTrain.setPositionDrive(positions, velocity, tolerance);
            }
        }
        lastTime = runtime.seconds();
    }

    @Override
    public void stop() {
        PathServer.stopServer();
    }
}
