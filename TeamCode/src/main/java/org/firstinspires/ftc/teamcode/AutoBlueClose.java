package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue Close", group="Robot")
public class AutoBlueClose extends LinearOpMode {
    final ElapsedTime runtime = new ElapsedTime();
    int launcherVelocity = 770;

    @Override
    public void runOpMode() {

        LinearOpMode o = this;
        RockinBot r = new RockinBot(o, "Shooter");     // Passing in code from RockinBot

        telemetry.addData("Autonomous Ready", "You can press start now");
        telemetry.addData("This code was last updated", "12/8/2025, 11:15 am"); // Todo: Update this date when the code is updated
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        r.getPinpointPosition();
        r.launcherVelocity(launcherVelocity);

        shootBalls(r); // Shoot preloaded balls

        // Pick up 1st set of balls
        r.driveToPos(-780, -900, 0, 3);
        r.turnLifterToDegrees(0, 600);
        r.intakePower(0.8);
        r.driveToPos(-780, 0, 0, 2);
        r.waitForLifter(1.0);

        shootBalls(r);

        // Pick up 2nd set of balls
        r.driveToPos(-1400, -900, 0, 3);
        r.turnLifterToDegrees(0, 500);
        r.intakePower(0.8);
        r.driveToPos(-1400, 200, 0, 2);
        r.waitForLifter();

        // move to avoid gate
        r.driveToPos(-1400, -300, 0);

        shootBalls(r);

        // park
        r.driveToPos(100, -800, -35);
        r.launcherVelocity(0);
        r.intakePower(0);
    }

    private void shootBalls(RockinBot r)
    {
        r.intakePower(0.2);
        r.driveToPos(-150, -750, -35);
        r.intakePower(0.5);
        r.waitForLaunchers(launcherVelocity);
        r.turnLifterToDegrees(90);
        r.waitForLifter();
        r.waitForLaunchers(launcherVelocity);
        r.turnLifterToDegrees(180);
        r.waitForLifter();
        r.waitForLaunchers(launcherVelocity);
        r.turnLifterToDegrees(360);
        r.waitForLifter();
        r.turnLifterToDegrees(-180);
    }
}