package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red Close", group="Robot")
public class AutoRedClose extends LinearOpMode {

    private double launcherVelocity = 870; // Change this when changing launcher velocity

    @Override
    public void runOpMode() {
        LinearOpMode o = this;
        RockinBot r = new RockinBot(o,"Shooter");     // Passing in code from RockinBot

        telemetry.addData("Autonomous Ready", "You can press start now");
        telemetry.addData("This code was last updated", "1/30/2026 5:58 pm"); // Todo: Update this date when the code is updated
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        r.getPinpointPosition();
        r.intakePower(0.2);
        r.launcherVelocity(launcherVelocity);

        // 1st set of balls
        shootBalls(r);

        // 2nd set of balls
        r.driveToPos(720, -700, 0, 20, 2, 2);
        r.driveToPos(720,  -50, 0, 20, 2, 2, true);
        shootBalls(r);

        // 3rd set of balls
        r.driveToPos(1200, -700, 0, 10, 2, 3);
        r.driveToPos(1200,  110, 0, 10, 2, 2, true);
        r.driveToPos(1200, -250, 0, 50, 5, 1); // move to avoid gate
        r.driveToPos(1200,  -50, 0, 10, 2, 2); // empty gate
        shootBalls(r);

        // 4th set of balls
        r.driveToPos(1900, -700, 0, 20, 2, 3);
        r.driveToPos(1900,  110, 0, 20, 2, 2, true);
        r.driveToPos(1350, -600, 0, 40, 5, 1); // move to avoid gate
        shootBalls(r);

        // Park
        r.driveToPos(1200, -200, 0, 50, 5, 2);
        r.launcherVelocity(0);
        r.intakePower(0);
        r.lightsOff();
    }

    private void shootBalls(RockinBot r)
    {
        r.driveToPos(500, -700, 42); // Go to shooting position
        r.waitForLaunchers(launcherVelocity);
        r.intakePower(0.5);
        sleep(500);
        r.turnLifterToDegrees(360); // Shoot the balls
        r.waitForLifter();
        sleep(400);
        r.turnLifterToDegrees(-150); // Reset lifter
    }
}
