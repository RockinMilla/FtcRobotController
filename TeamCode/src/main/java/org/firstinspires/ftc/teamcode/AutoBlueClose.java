package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue Close", group="Robot")
public class AutoBlueClose extends LinearOpMode {

    private double launcherVelocity = 820;

    @Override
    public void runOpMode() {

        LinearOpMode o = this;
        RockinBot r = new RockinBot(o, "Shooter");     // Passing in code from RockinBot

        telemetry.addData("Autonomous Ready", "You can press start now");
        telemetry.addData("This code was last updated", "1/23/2026, 5:38 pm"); // Todo: Update this date when the code is updated
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        r.getPinpointPosition();
        r.intakePower(0.2);
        r.launcherVelocity(launcherVelocity);
        
        // 1st set of balls
        shootBalls(r);

        // 2nd set of balls
        r.driveToPos(-780, -700, 0, 10, 2, 3);
        r.driveToPos(-780,  -50, 0, 10, 2, 2, true); // Pick up
        shootBalls(r);

        // 3rd set of balls
        r.driveToPos(-1400, -700, 0, 20, 2, 3);
        r.driveToPos(-1400,  150, 0, 20, 2, 2, true); // Pick up
        sleep(500);
        r.driveToPos(-1200, -300, 0); // move to avoid gate
        r.driveToPos(-1200,    0, 0); // empty gate
        shootBalls(r);

        // 4th set of balls
        r.driveToPos(-1900, -700, 0, 20, 2, 4);
        r.driveToPos(-1920,  150, 0, 20, 2, 2, true); // Pick up
        sleep(500);
        r.driveToPos(-1350, -400, 10); // move to avoid gate
        shootBalls(r);

        // Park
        r.driveToPos(-1200, -400, 0, 30, 5, 2);
        r.launcherVelocity(0);
        r.intakePower(0);
        r.lightsOff();
    }

    private void shootBalls(RockinBot r)
    {
        r.driveToPos(-440, -700, -29, 23, 1, 5); // Go to shooting position
        r.waitForLaunchers(launcherVelocity);
        r.intakePower(0.5);
        sleep(500);
        r.turnLifterToDegrees(360); // Shoot the balls
        r.waitForLifter();
        sleep(400);
        r.turnLifterToDegrees(-150); // Reset lifter
    }
}
