package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ControlFreaks.CFPushBotHardware;


/**
 * Created by adevries on 11/6/2015.
 */
@Autonomous(name="blue1", group="Blue")
//@Disabled
public class CFPushBotAuto_blue1 extends LinearOpMode
{
    /* Declare OpMode members. */
    CFPushBotHardware robot   = new CFPushBotHardware();

    boolean v_useGyro = false;
    boolean v_turnSlow = true;

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
     * 1680 ticks in a 60 moters
     * 560 ticks in a 20 moters
     * 1120 ticks in a 40 moter
     */
    private int v_state = 0;

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
        int color = -1;
        robot.init(this);
        robot.blueled_on();
        //robot.redled_on();
        //robot.led7seg_timer_init(30);
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.hardware_loop();

            switch (v_state) {
                //
                // Synchronize the state machine and hardware.
                //
                case 0:
                    //
                    // drive Forward  inches
                    //
                    //robot.led7seg_timer_start(30);
                    robot.sensor_color_enable(true);
                    robot.sensor_color_led(true);
                    robot.jewel_extend();

                    v_state++;
                    break;
                case 1:
                //enable color sensor
                    robot.timewait(15);
                    // Transition to the next state when this method is called again.
                    v_state++;
                    break;
                case 2:
                    color = robot.sensor_color_GreatestColor();
                     if (color == 0 || color == 2)
                     {
                         robot.set_message("Color Read " + color);
                         v_state++;
                     }else{
                         if(robot.timewait_Complete()){
                             robot.set_message("Color Timeout ");
                             v_state = v_state+ 3;
                         }
                     }
                    break;
                case 3:
                    if (color == 0) {
                        robot.turn_degrees(20, true, v_useGyro);
                        v_state++;
                    }else if(color==2){
                        robot.turn_degrees(-20, true, v_useGyro);
                        v_state++;
                    }
                    break;
                case 4:
                    if(robot.turn_complete()){
                        v_state++;
                    }
                    break;
                case 5:
                    robot.jewel_retract();
                    v_state++;
                    break;
                case 6:
                    if (color == 0) {
                        robot.turn_degrees(-20,true,v_useGyro);
                    }else if(color==2){
                        robot.turn_degrees(20, true, v_useGyro);
                    }
                    //hi
                    v_state++;
                    break;
                case 7:
                    if(robot.turn_complete()){
                        v_state++;
                    }
                    break;
                case 8:
                    robot.drive_inches(24,v_useGyro);
                    v_state++;
                    break;
                case 9:
                    if(robot.drive_inches_complete())
                    {
                        v_state++;
                    }
                    break;
                case 10:
                    robot.turn_degrees(90,false,v_useGyro);
                    v_state++;
                    break;
                case 11:
                    if (robot.turn_complete())
                    {
                        v_state++;
                    }
                    break;
                case 12:
                    robot.drive_inches(16,v_useGyro);
                    v_state++;
                    break;
                case 13:
                    if (robot.drive_inches_complete())
                    {
                        v_state++;

                    }
                    robot.set_message("the robot worked thank connor ");
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



}