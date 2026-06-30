package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.localizer.PoseEstimator;
import org.firstinspires.ftc.teamcode.localizer.pinpoint.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.path.Path;
import org.firstinspires.ftc.teamcode.path.PathFollower;
import org.firstinspires.ftc.teamcode.path.points.Waypoint;
import org.firstinspires.ftc.teamcode.util.math.units.Pose2d;

@Autonomous
public class AutoOpMode extends LinearOpMode {
    MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap, new PoseEstimator(new PinpointLocalizer(hardwareMap)));
    PathFollower pathFollower = new PathFollower(drivetrain);

    @Override
    public void runOpMode() {
        Path[] paths = new Path[3];

        paths[0] = new Path(new Waypoint(new Pose2d(0, 0, 0)))
                .addWaypoint(new Waypoint(new Pose2d(15, -5, 0)));

        paths[1] = new Path(new Waypoint(new Pose2d(15, -5, 0)))
                .addWaypoint(new Waypoint(new Pose2d(20, 15, 0)));

        paths[2] = new Path(new Waypoint(new Pose2d(20, 15, 0)))
                .addWaypoint(new Waypoint(new Pose2d(0, 0, 0)));

        short currentPath = 0;

        waitForStart();

        while (opModeIsActive()) {
            if (pathFollower.isFinished()) {
                pathFollower.startPath(paths[currentPath]);
                currentPath ++;
            }

            pathFollower.update();
        }
    }
}
