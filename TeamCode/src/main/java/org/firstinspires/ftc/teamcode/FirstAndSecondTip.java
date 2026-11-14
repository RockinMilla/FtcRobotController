package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name="First and Second Tip Auto", group="Robot")
public class FirstAndSecondTip extends LinearOpMode {
    private static final double LEFT_SHOOTER_VELOCITY = 1775;
    private static final double RIGHT_SHOOTER_VELOCITY = 1775;
    private static final double LEFT_PVALUE = 100;
    private static final double RIGHT_PVALUE = 100;
    private static final double TRIGGER_POWER = 1;
    private static final double INTAKE_POWER = 0.8;

    @Override
    public void runOpMode() {
        RockinBot robot = new RockinBot(this);

        telemetry.addData("Autonomous Ready", "Shoot, then park");
        telemetry.addData("Version", "v20261113-07");
        telemetry.update();
        waitForStart();

        // Start motors
        robot.shooterVelocity(LEFT_SHOOTER_VELOCITY, RIGHT_SHOOTER_VELOCITY);
        robot.setLeftPValue(LEFT_PVALUE);
        robot.setRightPValue(RIGHT_PVALUE);
        sleep(3000); // let shooters get up to speed

        // Shoot 1st round: balls 1, 2, 3, & 4
        robot.triggerPower(TRIGGER_POWER);
        sleep(1000);
        robot.intakePower(INTAKE_POWER);
        sleep(1000);
        robot.triggerPower(0);

        // Collect balls from corner
        robot.driveToPos(0, -150, 0, 25, 5, 1, 0.5); // Waypoint to get off the wall
        robot.driveToPos(400, -20, -85, 50,2, 3, 0.7); // Move to collect balls
        robot.driveToPos(1100, 0, -85, 50, 2, 2, 0.5); // Collect them
        sleep(500);

        // Shoot 2nd round: balls 5, 6, 7, & 8
        robot.driveToPos(100, -2000, 180, 300, 5, 3); // Waypoint to avoid the hive
        robot.driveToPos(-200, -2650, 180, 200, 5, 2); // Waypoint to avoid the flower
        robot.driveToPos(-400, -2800, 180, 20, 1, 3, 0.5); // Go to new shooting position
        sleep(100);
        robot.triggerPower(TRIGGER_POWER);
        sleep(1500);
        robot.triggerPower(0);
        sleep(200);

        robot.driveToPos(-80, -2350, -180, 20, 1, 2, 0.5); // get close
        robot.setLumberjackPositions(0.1, 0.67); //down
        sleep(400);
        robot.driveToPos(-80, -2700, 180, 20, 1, 2, 0.3); // collect balls
        sleep(1300);
        robot.driveToPos(-80, -2350, -180, 20, 1, 2, 0.5); // move back
        sleep(400);
        robot.setLumberjackPositions(0.73, 0.03); //up

        // Shoot 4th round: balls 9, 10, 11, & 12
        robot.driveToPos(-400, -2400, 180, 100, 3, 2); // Waypoint to avoid the flower
        robot.driveToPos(-400, -2800, 180, 20, 1, 3, 0.7); // Go to shooting position
        sleep(100);
        robot.triggerPower(TRIGGER_POWER);
        sleep(1200);
        robot.triggerPower(0);
        sleep(300);

        // Park
        //RobotLog.vv("Rockin' Robots", "Drive to the parking box");
        robot.driveToPos(-400, -2700, 180, 50, 1, 3);
        robot.driveToPos(670, -2500, 180, 20, 2, 4, 0.5);
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