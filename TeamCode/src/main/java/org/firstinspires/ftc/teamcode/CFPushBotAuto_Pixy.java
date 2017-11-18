package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ControlFreaks.CFPushBotHardware;
import org.firstinspires.ftc.teamcode.ControlFreaks.PixyCamera;


/**
 * Created by adevries on 11/6/2015.
 */
@Autonomous(name="Pixy", group="MrD")
//@Disabled
public class CFPushBotAuto_Pixy extends LinearOpMode
{
    /* Declare OpMode members. */
        CFPushBotHardware robot   = new CFPushBotHardware();
        byte red = (byte)255;
        byte green = (byte)0;
        byte blue = (byte)0;



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
        robot.setup_am20();
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
                    robot.sensor_pixy_init();
                    v_state++;
                    break;
                case 1:

                    //
                    // Transition to the next state when this method is true
                    if (robot.sensor_pixy_set_leds((byte)0,(byte)255,(byte)0)) {
                        //
                        robot.set_message("Set Led Color");
                    }
                    v_state++;

                    break;
                case 2:
                    robot.sensor_pixy_enable(true);
                    v_state++;
                    break;
                case 3:
                    PixyCamera.Block myBlock = robot.sensor_pixy_get_LargestBlock();
                    if(myBlock != null){
                        //for(int i=0; i< myBlocks.length; i ++){
                        //    PixyCamera.Block myBlock = myBlocks[i];
                            robot.set_message(myBlock.print());
                        //}
                        v_state++;
                    }else
                    {
                        robot.set_message("Blocks is null");
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
