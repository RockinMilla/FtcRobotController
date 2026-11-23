package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// so we can update them quickly and easily.
@Autonomous(name="Second Tip Auto", group="Robot")
public class SecondTipOnly extends LinearOpMode {

    @Override
    public void runOpMode() {
        RockinBot r = new RockinBot(this);

        telemetry.addData("Autonomous Ready", "Shoot twice, then park");
        telemetry.addData("Version", "v20261112-08");
        telemetry.update();
        waitForStart();

        sleep(5000); // wait for other team

        // Start motors
        r.shooterVelocity(r.LEFT_SHOOTER_VELOCITY, r.RIGHT_SHOOTER_VELOCITY);
        r.setPValues(r.LEFT_PVALUE, r.RIGHT_PVALUE);
        sleep(3000); // let shooters get up to speed

        // Shoot 1st round: balls 1, 2, 3, & 4
        r.triggerPower(r.TRIGGER_POWER);
        sleep(800);
        r.intakePower(r.INTAKE_POWER);
        sleep(800);
        r.triggerPower(0);

        // Collect balls from flower
        r.intakePower(1);
        r.driveToPos(-400, -500, 0, 25, 5, 1, 0.6); // Waypoint to get off the wall
        r.setLumberjackDown();
        sleep(500);
        r.driveToPos(-400, -200, 0, 10, 2, 2, 0.4); // Collect them
        sleep(1000);
        r.driveToPos(-400, -500, 0, 25, 5, 1, 0.6); // Waypoint to get off the wall
        r.setLumberjackUp(); //up

        // Shoot 2nd round: balls 5, 6, 7, & 8
        r.driveToPos(0, -500, 0, 50, 5, 1);
        r.driveToPos(0, 0, 0, 20, 1, 3, 0.7);
        r.triggerPower(r.TRIGGER_POWER);
        sleep(800);
        r.intakePower(r.INTAKE_POWER);
        sleep(800);
        r.triggerPower(0);
        r.intakePower(0);

        // Park
        r.driveToPos(0, -300, 0, 50, 5, 1);
        r.driveToPos(-1100, -300, 0, 50, 2, 4, 0.7);
    }
}