package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name="Red Close", group="Robot")
public class AutoRedClose extends LinearOpMode {
    final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        LinearOpMode o = this;
        RockinBot r = new RockinBot(o,"Shooter");     // Passing in code from RockinBot

        telemetry.addData("Autonomous Ready", "You can press start now");
        telemetry.addData("This code was last updated", "1/4/2026, 2:49 pm"); // Todo: Update this date when the code is updated


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        int launcherSpeed = 870;

        r.getPinpointPosition();
        r.launcherVelocity(launcherSpeed);

        shootBalls(r, launcherSpeed); // Shoot preloaded balls

        // Pick up 1st set of balls
        r.driveToPos(720, -900, 0, 15, 3, 4);
        r.waitForLifter();
        r.turnLifterToDegrees(0, 600);
        r.intakePower(0.8);
        r.driveToPos(720, 0, 0, 15, 3, 3);
        r.waitForLifter();
        sleep(100);

        shootBalls(r, launcherSpeed);

        // Pick up 2nd set of balls
        r.driveToPos(1320, -900, 0, 15, 3, 4);
        r.waitForLifter();
        r.turnLifterToDegrees(0, 600);
        r.intakePower(0.8);
        r.driveToPos(1370, 200, 0, 15, 3, 3);
        r.waitForLifter();
        sleep(100);

        r.driveToPos(1400, -300, 0); // move to avoid gate

        r.turnLifterByDegrees(-10, 700);
        shootBalls(r, launcherSpeed);
        r.driveToPos(-200, -800, 37); // park
        r.launcherVelocity(0);
        r.intakePower(0);
    }

    private void shootBalls(RockinBot r, int launchVeloc)
    {
        r.intakePower(0.2);
        r.driveToPos(100, -700, 37);
        r.intakePower(0.5);
        r.turnLifterToDegrees(80, 1400);
        r.waitForLaunchers(launchVeloc);
        r.turnLifterToDegrees(180, 2000);
        sleep(200);
        r.waitForLaunchers(launchVeloc);
        r.turnLifterToDegrees(360, 1400);
        r.waitForLifter();
        sleep(100);
        r.turnLifterToDegrees(-190); // Reset lifter
    }
}