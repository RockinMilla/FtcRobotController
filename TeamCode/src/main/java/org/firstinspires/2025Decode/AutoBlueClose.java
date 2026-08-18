package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue Close", group="Robot")
public class AutoBlueClose extends LinearOpMode {

    private double launcherVelocity = 870;

    @Override
    public void runOpMode() {
        LinearOpMode o = this;
        RockinBot r = new RockinBot(o, "Shooter");     // Passing in code from RockinBot

        telemetry.addData("Autonomous Ready", "You can press start now");
        telemetry.addData("This code was last updated", "5/15/2026, 1:45 pm"); // Todo: Update this date when the code is updated
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        r.getPinpointPosition();
        r.setpValue(50);
        r.intakePower(0.3);
        r.launcherVelocity(launcherVelocity);
        
        // 1st set of balls
        shootBalls(r);

        // 2nd set of balls
        r.driveToPos(-780, -700, 0, 10, 2, 3, false, true);
        r.driveToPos(-780,  -50, 0, 10, 2, 2, true); // Pick up
        sleep(200);
        shootBalls(r);

        // 3rd set of balls
        r.driveToPos(-1350, -700, 0, 20, 2, 3, false);
        r.driveToPos(-1400,  150, 0, 20, 2, 2, true); // Pick up
        sleep(200);
        r.driveToPos(-1230, -300, 0, 20, 2, 1, false, true); // move to avoid gate
        r.driveToPos(-1230,    0, 0, 10, 1, 1); // empty gate
        sleep(100);
        shootBalls(r);

        // 4th set of balls
        r.driveToPos(-1900, -700, 0, 20, 2, 4, false, false);
        r.driveToPos(-1920,  150, 0, 20, 2, 2, true); // Pick up
        sleep(200);
        shootBalls(r);

        // Park
        r.driveToPos(-1200, -300, 0, 30, 5, 2);
        r.launcherVelocity(0);
        r.intakePower(0);
        r.lightsOff();
    }

    private void shootBalls(RockinBot r)
    {
        r.driveToPos(-440, -700, -37, 10, 1, 3); // Go to shooting position
        r.waitForLaunchers(launcherVelocity);
        r.intakePower(0.5);
        r.turnLifterToDegrees(360); // Shoot the balls
        r.waitForLifter();
        r.turnLifterToDegrees(-150, 3000); // Reset lifter
    }
}
