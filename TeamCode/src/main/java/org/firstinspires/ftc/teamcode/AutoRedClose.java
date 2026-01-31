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
        telemetry.addData("This code was last updated", "1/30/2026 5:58 pm"); // Todo: Update this date when the code is updated
        telemetry.update();

        waitForStart(); // Wait for the game to start (driver presses PLAY)

        int launcherSpeed = 830; // Setting launcher speed once

        r.getPinpointPosition();
        r.launcherVelocity(launcherSpeed);

        shootBalls(r); // Shoot 1st set of balls

        // Pick up 2nd set of balls
        r.driveToPos(720, -900, 0, 20, 3, 2);
        r.waitForLifter();
        r.turnLifterToDegrees(0, 550);
        r.intakePower(0.8);
        r.driveToPos(720, -50, 0, 15, 3, 2);
        r.driveToPos(720, -50, 0, 15, 3, 1); // Try again
        r.waitForLifter();
        sleep(100);

        shootBalls(r); // Shoot 2nd set of balls

        // Pick up 3rd set of balls
        r.driveToPos(100, -700, 0, 15, 3, 1); // Turn first
        r.driveToPos(1200, -900, 0, 15, 3, 3);
        r.waitForLifter();
        r.turnLifterToDegrees(0, 400);
        r.intakePower(0.8);
        r.driveToPos(1340, 110, 0, 15, 3, 1.5);
        r.waitForLifter();
        sleep(100);

        r.driveToPos(1200, -250, 0, 30 ,5, 1); // move to avoid gate
        r.driveToPos(1200, -50, 0, 15, 3, 2); // empty gate
        sleep(100);
        shootBalls(r); // Shoot 3rd set of balls

        // Pick up 4th set of balls
        r.driveToPos(100, -800, 0, 10, 2, 3);
        r.driveToPos(1700,-800,0,10,2,3);
        r.intakePower(0.8);
        //sleep(100);
        r.turnLifterToDegrees(0,500);
        r.driveToPos(1900,110,0,10,2,1.5);
        r.waitForLifter();
        sleep(100);

        r.driveToPos(1350, -600, 0, 40, 5, 1); // move to avoid gate
        shootBalls(r); // Shoot 4th set of balls

        r.driveToPos(1200, -400, 0, 30, 3, 2); // Park
        r.driveToPos(1200, -400, 0, 20, 3, 1); // Park again

        r.launcherVelocity(0);
        r.intakePower(0);
    }

    private void shootBalls(RockinBot r)
    {
        r.intakePower(0.4);
        r.driveToPos(100, -700, 38); // Go to shooting position
        r.driveToPos(100, -700, 38, 20, 3, 1); // Just in case
        r.intakePower(0.9); // Propell balls
        r.turnLifterToDegrees(360, 1400); // Shoot
        r.waitForLifter();
        sleep(100); // Necessary to ensure that all balls are shot
        r.turnLifterToDegrees(-190); // Reset lifter
    }
}