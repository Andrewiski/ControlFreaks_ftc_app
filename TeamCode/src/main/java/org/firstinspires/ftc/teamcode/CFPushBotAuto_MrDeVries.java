package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ControlFreaks.CFPushBotHardware;


/**
 * Created by adevries on 11/6/2015.
 */
@Autonomous(name="Mr D Turn -90 AM20", group="MrD")
@Disabled
public class CFPushBotAuto_MrDeVries extends LinearOpMode
{
    /* Declare OpMode members. */
        CFPushBotHardware robot   = new CFPushBotHardware();



    //--------------------------------------------------------------------------
    //
    // loop
    //
    /**
     * Implement a state machine that controls the robot during auto-operation.
     * The state machine uses a class member and encoder input to transition
     * between states.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(this);
        robot.blueled_on();
        robot.setupAutoDrive();
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.hardware_loop();
            switch (v_state) {
                //
                // Synchronize the state machine and hardware.
                //
                case 0:
                    //v_state = 100;
                    robot.turn_degrees(-90, false,false);
                    v_state++;
                    break;
                case 1:

                    //
                    // Transition to the next state when this method is true
                    if (robot.turn_complete()) {
                        //
                        v_state++;

                    }

                    break;

                default:
                    //
                    // The autonomous actions have been accomplished (i.e. the state has
                    // transitioned into its final state.
                    //
                    break;
            }



        } // loop
    }
    //--------------------------------------------------------------------------
    //
    // v_state
    //
    /**
     * This class member remembers which state is currently active.  When the
     * start method is called, the state will be initialized (0).  When the loop
     * starts, the state will change from initialize to state_1.  When state_1
     * actions are complete, the state will change to state_2.  This implements
     * a state machine for the loop method.
     */
    private int v_state = 0;


}
