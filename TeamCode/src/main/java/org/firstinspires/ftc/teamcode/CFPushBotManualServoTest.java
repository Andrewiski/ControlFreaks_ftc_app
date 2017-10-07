package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ControlFreaks.CFPushBotHardware;

/**
 * Created by adevries on 11/6/2015.
 */
@TeleOp(name="Servo Test", group="Manual")  // @Autonomous(...) is the other common choice
//@Disabled
public class CFPushBotManualServoTest extends LinearOpMode {

    /* Declare OpMode members. */
    CFPushBotHardware robot;   // Use a Pushbot's hardware


    //--------------------------------------------------------------------------
    //
    // loop
    //
    /**
     * Implement a state machine that controls the robot during
     * manual-operation.  The state machine uses gamepad input to transition
     * between states.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */
    float stickdeadzone = .1f;
    boolean buttonXReleased = true;
    boolean buttonBReleased = true;
    boolean buttonAReleased = true;
    boolean buttonDpadUpReleased = true;
    boolean buttonDpadDownReleased = true;
    boolean buttonLeftBumperReleased = true;
    boolean buttonRightBumperReleased = true;
    boolean buttonG2XReleased = true;
    boolean buttonG2YReleased = true;
    boolean buttonG2AReleased = true;
    boolean buttonG2BReleased = true;
    ElapsedTime myFliperRetractElapsedTime;
    @Override
    public void runOpMode() throws InterruptedException {

        try {
            /* Declare OpMode members. */
            robot           = new CFPushBotHardware();   // Use a Pushbot's hardware
            robot.init(this);

            robot.setupManualDrive();
            //robot.vuforia_Init();
            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                try {
                    robot.hardware_loop();
                    if(gamepad1.left_trigger > .2 || gamepad1.right_trigger > .2 ) {
                        robot.elbow_setposition(-gamepad1.left_stick_y);
                    }else{
                        robot.elbow_setposition(-gamepad1.left_stick_y/1.5f);
                    }
                    if(gamepad1.left_bumper){
                        if(buttonLeftBumperReleased == true) {
                            robot.elbow_retract();
                            buttonLeftBumperReleased = false;
                        }
                    }else{
                        buttonLeftBumperReleased = true;
                    }
                    if(gamepad1.right_bumper){
                        if(buttonRightBumperReleased == true) {
                            robot.elbow_extend();
                            buttonRightBumperReleased = false;
                        }
                    }else{
                        buttonRightBumperReleased = true;
                    }
                    if(gamepad2.dpad_up) {
                        robot.elbow_step(.05);
                    }
                    if(gamepad2.dpad_down){
                        robot.elbow_step(-.05);
                    }

                    //robot.waitForTick(2);
                }catch(Exception ex){
                    robot.set_error_message("Fatal Error "  + ex.toString());
                }
            } // loop

        }catch(Exception ex){
            robot.set_error_message("Fatal Error "  + ex.toString());
        }
    }


}
