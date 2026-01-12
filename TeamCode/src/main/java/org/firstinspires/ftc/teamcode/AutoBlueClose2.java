package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue Close 2", group="Robot")
public class AutoBlueClose2 extends LinearOpMode {
    final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        LinearOpMode o = this;
        RockinBot r = new RockinBot(o,"Shooter");     // Passing in code from RockinBot

        telemetry.addData("Autonomous Ready", "You can press start now");
        telemetry.addData("This code was last updated", "1/11/2026, 2:40 pm"); // Todo: Update this date when the code is updated
        telemetry.update();

        waitForStart(); // Wait for the game to start (driver presses PLAY)

        int launcherSpeed = 830; // Setting launcher speed once

        r.getPinpointPosition();
        r.launcherVelocity(launcherSpeed);
        r.intakePower(0.5);

        shootBalls(r); // Shoot preloaded balls

        // Pick up 1st set of balls
        r.driveToPos(-720, -900, 0, 15, 4, 4);
        r.waitForLifter();
        r.intakePower(0.7);
        r.driveToPos(-720, -730, 0, 15, 4, 1.6);
        r.turnLifterToDegrees(0, 800);
        r.driveToPos(-720, 160, 0, 15, 4, 1.6);
        r.waitForLifter();
        sleep(100);

        shootBalls(r);

        // Pick up 2nd set of balls
        r.driveToPos(-1370, -900, 0, 15, 4, 4);
        r.waitForLifter();
        r.intakePower(0.7);
        r.driveToPos(-1370, -730, 0, 15, 4, 1.6);
        r.turnLifterToDegrees(0, 800);
        r.driveToPos(-1370, 205, 0, 15, 4, 3.5);
        r.waitForLifter();
        sleep(100);

        r.driveToPos(-1400, -300, 0); // move to avoid gate

        shootBalls(r);

        // Pick up 3rd set of balls
        r.driveToPos(-1880,-900,1,15,4,6);
        r.waitForLifter();
        r.intakePower(0.7);
        r.driveToPos(-1880, -730, 0, 15, 4, 1.6);
        r.turnLifterToDegrees(0, 800);
        r.driveToPos(-1880,120,1,15,4,3.3);
        r.waitForLifter();
        sleep(100);

        r.launcherVelocity(0);
        r.intakePower(0);
        r.driveToPos(-1200, -300, 0, 30, 4, 3); // Park
    }

    private void shootBalls(RockinBot r)
    {
        r.intakePower(0.2);
        r.driveToPos(-100, -700, -39); // Go to shooting position
        r.intakePower(0.5); // Propell balls
        r.turnLifterToDegrees(360, 1400);
        r.waitForLifter();
        sleep(200); // Necessary to ensure that all balls are shot
        r.turnLifterToDegrees(-190); // Reset lifter
    }
}