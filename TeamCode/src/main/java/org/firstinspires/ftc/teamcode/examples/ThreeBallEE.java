package org.firstinspires.ftc.teamcode.examples;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointOdometry;
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;
import org.firstinspires.ftc.teamcode.system.PathServer;

import java.util.Arrays;


// This is an example OpMode designed to interface with the Overlake Robotics Path Planner.
// It will let you use the path planner to upload a path to the robot with movement and two tags.
// The two tags are:
//      - velocity: Changes the velocity of the robot to the tags value (this is in in/s).
//      - pause: Pauses the robot for a number of seconds equal to the tags value.
// Feel free to take this OpMode and add your own robot specific tags if you are using the path planner.
@Config
@Autonomous(name = "3 Ball", group = "Autonomous")
public class ThreeBallEE extends OpMode {

    public double yOffset = -168.0; //tune
    public double xOffset = -84.0;  //tune

    private OdometryHolonomicDrivetrain driveTrain;
    public Pose2D[] positions;
    public PathServer.Tag[] tags;

    private int lastTagIndex = 0;
    private final ElapsedTime runtime = new ElapsedTime();
    private double lastTime = 0;
    private double pauseTimeLeft = 0;
    private int pausedIndex = -1;

    // Mechanisms
    public DcMotorEx intake;
    public DcMotorEx turret;
    public DcMotorEx shooter1, shooter2;

    // Shooting state machine
    private boolean shooting = false;
    private double shootTimer = 0;
    private int shootStage = 0;

    // Vision
    private boolean scanning = false;
    public Limelight3A limelight;
    public double tx = 0;
    public double ty = 0;

    // Limelight aiming logic
    public void scanning() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            tx = result.getTx();
            ty = result.getTy();

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }

        // Rotate turret until target is centered
        if (tx > 5) {
            turret.setPower(0.4);
        } else if (tx < -5) {
            turret.setPower(-0.4);
        } else {
            turret.setPower(0);
        }
    }

    @Override
    public void init() {
        GoBildaPinpointDriver pinpointDriver =
                hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpointDriver.setOffsets(xOffset, yOffset, DistanceUnit.MM);

        driveTrain = new OdometryHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                new GoBildaPinpointOdometry(pinpointDriver)
        );

        // Hardware initialization
        intake = hardwareMap.get(DcMotorEx.class, "i");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        shooter1 = hardwareMap.get(DcMotorEx.class, "sf1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "sf2");

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(2);
        limelight.start();

        PathServer.startServer();
    }

    @Override
    public void init_loop() {
        driveTrain.updatePosition();
        PathServer.setRobotPose(driveTrain.getPosition());
    }

    @Override
    public void start() {
        driveTrain.setVelocity(
                (int) (PathServer.getVelocity()
                        * BasicHolonomicDrivetrain.FORWARD_COUNTS_PER_INCH)
        );

        positions = PathServer.getPath();
        driveTrain.setPosition(PathServer.getStartPose());
        driveTrain.setTolerance(PathServer.getTolerance());

        tags = PathServer.getTags();
        Arrays.sort(tags);

        driveTrain.setPositionDrive(positions);
        runtime.reset();
    }

    @Override
    public void loop() {
        driveTrain.updatePosition();
        Pose2D pos = driveTrain.getPosition();
        PathServer.setRobotPose(pos);

        double curTime = runtime.seconds();
        double dt = curTime - lastTime;
        lastTime = curTime;

        // Run aiming continuously when enabled
        if (scanning) {
            scanning();
        }

        // Shooting state machine (non-blocking)
        if (shooting) {
            double elapsed = runtime.seconds() - shootTimer;

            switch (shootStage) {
                case 0: // spin up shooter
                    shooter1.setPower(1.0);
                    shooter2.setPower(1.0);
                    if (elapsed >= 2.0) {
                        shootStage++;
                        shootTimer = runtime.seconds();
                    }
                    break;

                case 1: // feed ring 1
                    intake.setPower(-1);
                    if (elapsed >= 1.0) {
                        shootStage++;
                        shootTimer = runtime.seconds();
                    }
                    break;

                case 2: // feed ring 2
                    intake.setPower(-1);
                    if (elapsed >= 1.0) {
                        shootStage++;
                    }
                    break;

                case 3: // stop everything
                    shooter1.setPower(0);
                    shooter2.setPower(0);
                    intake.setPower(0);
                    shooting = false;
                    break;
            }
        }

        if (pauseTimeLeft <= 0) {
            driveTrain.drive();
            int nextPointIndex = driveTrain.getNextPointIndex();

            while (lastTagIndex < tags.length &&
                    tags[lastTagIndex].index <= nextPointIndex) {

                PathServer.Tag currTag = tags[lastTagIndex];

                switch (currTag.name) {
                    case "velocity":
                        driveTrain.setVelocity(
                                (int) (currTag.value
                                        * BasicHolonomicDrivetrain.FORWARD_COUNTS_PER_INCH)
                        );
                        break;

                    case "pause":
                        if (currTag.value <= 0) break;
                        pauseTimeLeft = currTag.value;
                        pausedIndex = nextPointIndex;
                        driveTrain.setPositionDrive(positions[nextPointIndex - 1]);
                        break;

                    case "intake":
                        intake.setPower(currTag.value > 0 ? -1 : 0);
                        break;

                    case "shoot":
                        if (currTag.value > 0 && !shooting) {
                            shooting = true;
                            shootStage = 0;
                            shootTimer = runtime.seconds();
                        }
                        break;

                    case "aim":
                        scanning = currTag.value > 0;
                        break;
                }

                lastTagIndex++;
            }
        } else {
            pauseTimeLeft -= dt;

            if (pauseTimeLeft <= 0) {
                pauseTimeLeft = 0;
                driveTrain.setPositionDrive(positions, pausedIndex);
            } else {
                driveTrain.setPositionDrive(positions[pausedIndex - 1]);
            }
        }
    }

    @Override
    public void stop() {
        PathServer.stopServer();
    }
}