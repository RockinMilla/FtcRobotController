package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue Close", group="Robot")
public class AutoBlueClose extends LinearOpMode {
    final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        LinearOpMode o = this;
        RockinBot r = new RockinBot(o, "Shooter");     // Passing in code from RockinBot

        telemetry.addData("Autonomous Ready", "You can press start now");
        telemetry.addData("This code was last updated", "1/10/2026, 2:41 pm"); // Todo: Update this date when the code is updated
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        r.getPinpointPosition();
        //r.intakePower(0.5);
        r.launcherVelocity(820);

        shootBalls(r); // Shoot preloaded balls

        // Pick up 1st set of balls
        r.driveToPos(-780, -900, 0, 16, 3, 3);
        r.intakePower(0.8);
        r.driveToPos(-780, -600, 0, 16,3,3);
        r.turnLifterToDegrees(0, 700);
        r.driveToPos(-780, -100, 0, 16, 3, 2);
        r.waitForLifter();
        sleep(100);

        shootBalls(r);

        // Pick up 2nd set of balls
        r.driveToPos(-1350, -950, 0, 16, 3, 3);
        r.intakePower(0.8);
        r.driveToPos(-1350, -650, 0, 16,3,3);
        r.turnLifterToDegrees(0, 600);
        r.driveToPos(-1350, 100, 0, 16, 3, 1.5);
        r.waitForLifter();
        sleep(100);

        r.driveToPos(-1400, -300, 0); // move to avoid gate

        shootBalls(r);

        // Pick up 3rd set of balls
        r.driveToPos(-1860,-900,0,15,3,6);
        r.waitForLifter();
        r.turnLifterToDegrees(0,600);
        r.intakePower(0.8);
        r.driveToPos(-1860,100,0,15,3,3);
        r.waitForLifter();
        sleep(100);

        r.launcherVelocity(0);
        r.intakePower(0);
        r.driveToPos(-1200, -300, 0, 30, 3, 3); // Park
    }

    private void shootBalls(RockinBot r)
    {
        r.intakePower(0.2);
        r.driveToPos(-100, -700, -36, 20, 2, 5); // Go to shooting position
        r.intakePower(0.5); // Propell balls
        r.turnLifterToDegrees(360, 1400); // Shoot
        r.waitForLifter();
        sleep(100); // Necessary to ensure that all balls are shot
        r.turnLifterToDegrees(-190); // Reset lifter
    }
}