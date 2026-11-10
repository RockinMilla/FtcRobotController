package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name="Second Tip Auto", group="Robot")
public class SecondTipOnly extends LinearOpMode {
    private static final double LEFT_SHOOTER_VELOCITY = 1775;
    private static final double RIGHT_SHOOTER_VELOCITY = 1800;
    private static final double TRIGGER_POWER = 1;
    private static final double INTAKE_POWER = 0.8;

    //Note: the lumberjack position stuff is commented out for now pls change to set left and right
    @Override
    public void runOpMode() {
        RockinBot robot = new RockinBot(this);

        telemetry.addData("Autonomous Ready", "Shoot twice, then park");
        telemetry.addData("Version", "v20261110-10");
        telemetry.update();
        waitForStart();

        // Start motors
        robot.shooterVelocity(LEFT_SHOOTER_VELOCITY, RIGHT_SHOOTER_VELOCITY);
        robot.setLeftPValue(70);
        robot.setRightPValue(100);
        sleep(4000); // let shooters get up to speed

        // Shoot 1st round: balls 1, 2, 3, & 4
        robot.triggerPower(TRIGGER_POWER);
        sleep(800);
        robot.intakePower(INTAKE_POWER);
        sleep(800);
        robot.triggerPower(0);

        // Collect balls from flower
        robot.driveToPos(-390, -500, 0, 25, 5, 1, 0.6); // Waypoint to get off the wall
        robot.setLumberjackPositions(0.1, 0.70); //down
        sleep(500);
        robot.driveToPos(-390, -200, 0, 10, 2, 2, 0.4); // Collect them
        sleep(1000);
        robot.driveToPos(-390, -500, 0, 25, 5, 1, 0.6); // Waypoint to get off the wall
        robot.setLumberjackPositions(0.73, 0.03); //up

        // Shoot 2nd round: balls 5, 6, 7, & 8
        robot.driveToPos(0, -500, 0, 50, 5, 1);
        robot.driveToPos(0, 0, 0, 20, 1, 3, 0.7);
        robot.triggerPower(TRIGGER_POWER);
        sleep(1600);
        robot.triggerPower(0);

        // Park
        //RobotLog.vv("Rockin' Robots", "Drive to the parking box");
        robot.driveToPos(0, -300, 0, 50, 5, 1);
        robot.driveToPos(-1100, -300, 0, 50, 2, 4, 0.7);
        robot.intakePower(0);

        robot.odo.update();
        Pose2D pose = robot.odo.getPosition();
        telemetry.addData("X (mm)", "%.1f", pose.getX(DistanceUnit.MM));
        telemetry.addData("Y (mm)", "%.1f", pose.getY(DistanceUnit.MM));
        telemetry.addData("Heading (degrees)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
    }
}