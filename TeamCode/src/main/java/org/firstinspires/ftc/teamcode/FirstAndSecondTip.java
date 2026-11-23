package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="First and Second Tip Auto", group="Robot")
public class FirstAndSecondTip extends LinearOpMode {

    @Override
    public void runOpMode() {
        RockinBot r = new RockinBot(this);

        telemetry.addData("Autonomous Ready", "Shoot, then park");
        telemetry.addData("Version", "v20261114-03");
        telemetry.update();
        waitForStart();

        // Start motors
        r.shooterVelocity(r.LEFT_SHOOTER_VELOCITY, r.RIGHT_SHOOTER_VELOCITY);
        r.setPValues(r.LEFT_PVALUE, r.RIGHT_PVALUE);
        sleep(3000); // let shooters get up to speed

        // Shoot 1st round: balls 1, 2, 3, & 4
        r.triggerPower(r.TRIGGER_POWER);
        sleep(1000);
        r.intakePower(r.INTAKE_POWER);
        sleep(1000);
        r.triggerPower(0);

        // Collect balls from corner
        r.driveToPos(0, -150, 0, 25, 5, 1, 0.5); // Waypoint to get off the wall
        r.driveToPos(400, -20, -85, 50,2, 3, 0.7); // Move to collect balls
        r.driveToPos(1100, 0, -85, 50, 2, 2, 0.5); // Collect them
        sleep(500);

        // Shoot 2nd round: balls 5, 6, 7, & 8
        r.driveToPos(100, -2000, 180, 300, 5, 3); // Waypoint to avoid the hive
        r.driveToPos(-200, -2650, 180, 200, 5, 2); // Waypoint to avoid the flower
        r.driveToPos(-400, -2800, 180, 20, 1, 3, 0.5); // Go to new shooting position
        r.shootSequence();

        r.driveToPos(-80, -2350, -180, 20, 1, 2, 0.5); // get close
        r.setLumberjackDown(); //down
        sleep(400);
        r.driveToPos(-80, -2700, 180, 20, 1, 2, 0.3); // collect balls
        sleep(1300);
        r.driveToPos(-80, -2350, -180, 20, 1, 2, 0.5); // move back
        sleep(400);
        r.setLumberjackUp(); //up

        // Shoot 4th round: balls 9, 10, 11, & 12
        r.driveToPos(-400, -2400, 180, 100, 3, 2); // Waypoint to avoid the flower
        r.driveToPos(-400, -2800, 180, 20, 1, 3, 0.7); // Go to shooting position
        r.shootSequence();

        // Park
        r.driveToPos(-400, -2600, 180, 50, 1, 3);
        r.driveToPos(200, -2400, 180, 50, 5, 3);
        r.driveToPos(800, -2000, 180, 20, 2, 4, 0.5);
        r.intakePower(0);
    }
}