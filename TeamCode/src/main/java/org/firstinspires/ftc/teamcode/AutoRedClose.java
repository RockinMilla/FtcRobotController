package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Red Close", group="Robot")
public class AutoRedClose extends LinearOpMode {
    final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        LinearOpMode o = this;
        RockinBot r = new RockinBot(o,"Shooter");     // Passing in code from RockinBot

        telemetry.addData("Autonomous Ready", "You can press start now");
        telemetry.addData("This code was last updated", "1/5/2026, 9:29 am"); // Todo: Update this date when the code is updated

        waitForStart(); // Wait for the game to start (driver presses PLAY)

        int launcherSpeed = 840; // Setting launcher speed once

        r.getPinpointPosition();
        r.launcherVelocity(launcherSpeed);

        shootBalls(r, launcherSpeed); // Shoot preloaded balls

        // Pick up 1st set of balls
        r.driveToPos(720, -900, 0, 15, 3, 4);
        r.waitForLifter();
        r.turnLifterToDegrees(0, 600);
        r.intakePower(0.8);
        r.driveToPos(720, 0, 0, 15, 3, 2);
        r.waitForLifter();
        sleep(100);

        shootBalls(r, launcherSpeed);

        // Pick up 2nd set of balls
        r.driveToPos(1350, -900, 0, 15, 3, 4);
        r.waitForLifter();
        r.turnLifterToDegrees(0, 600);
        r.intakePower(0.8);
        r.driveToPos(1350, 160, 0, 15, 3, 2);
        r.waitForLifter();
        sleep(100);

        r.driveToPos(1400, -300, 0); // move to avoid gate

        r.turnLifterByDegrees(-10, 700); // prevent stuckages
        shootBalls(r, launcherSpeed);

        // Pick up 3rd set of balls
        r.driveToPos(1900,-900,0,15,3,4);
        r.waitForLifter();
        r.turnLifterToDegrees(0,600);
        r.intakePower(0.8);
        r.driveToPos(1900,160,0,15,3,2);
        r.waitForLifter();
        sleep(100);
        r.driveToPos(1900, -400, 0); // Get out of the way of the gate

        r.turnLifterByDegrees(-10, 700);
        shootBalls(r, launcherSpeed);
        r.driveToPos(-200, -800, 37); // park
        r.launcherVelocity(0);
        r.intakePower(0);
    }

    private void shootBalls(RockinBot r, int launchVeloc)
    {
        r.intakePower(0.2);
        r.driveToPos(100, -700, 37); // Go to shooting position
        r.intakePower(0.5); // Propell balls
        r.turnLifterToDegrees(360, 1400); // Shoot
        r.waitForLifter();
        sleep(100); // Necessary to ensure that all balls are shot
        r.turnLifterToDegrees(-190); // Reset lifter
    }
}