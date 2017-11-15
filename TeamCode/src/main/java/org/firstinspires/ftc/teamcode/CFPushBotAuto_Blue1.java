package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ControlFreaks.CFPushBotHardware;


/**
 * Created by adevries on 11/6/2015.
 */
@Autonomous(name="Blue1", group="Blue")
//@Disabled
public class CFPushBotAuto_Blue1 extends LinearOpMode
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
        robot.drive_powerOverride(.25F,.25F, .25F);
        robot.turn_power_override(.25F, .25F);
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
                    robot.blockgrabber_close();
                    robot.jewel_lower();
                    robot.sensor_color_enable(true);
                    robot.sensor_color_led(true);

                    v_state++;
                    break;
                case 1:
                //enable color sensor
                    robot.timewait(3);
                    // Transition to the next state when this method is called again.
                    v_state++;
                    break;
                case 2:
                    if(robot.timewait_Complete()){
                        robot.lifter_step(300);
                        robot.timewait(5);
                        v_state++;
                    }
                    break;
                case 3:
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
                case 4:
                    if (color == 0) {
                        robot.drive_inches(3, v_useGyro);
                        v_state++;
                    }else if(color==2){
                        robot.drive_inches(-3,  v_useGyro);
                        v_state++;
                    }
                    break;
                case 5:
                    if(robot.drive_inches_complete()){
                        v_state++;
                    }
                    break;
                case 6:
                    robot.jewel_raise();
                    robot.timewait(3);
                    v_state++;
                    break;
                case 7:
                    if(robot.timewait_Complete()) {
                        v_state++;
                    }
                    break;
                case 8:
                    if (color == 0) {
                        robot.drive_inches(-3,v_useGyro);
                    }else if(color==2){
                        robot.drive_inches(3, v_useGyro);
                    }
                    v_state++;
                    //hi

                    break;
                case 9:
                    if(robot.drive_inches_complete()){
                        v_state++;
                    }
                    break;
                case 10:
                    robot.drive_inches(-30,v_useGyro);
                    v_state++;
                    break;
                case 11:
                    if(robot.drive_inches_complete())
                    {
                        v_state++;
                    }
                    break;
                case 12:
                    robot.turn_degrees(90,false,v_useGyro);
                    v_state++;
                    break;
                case 13:
                    if (robot.turn_complete())
                    {
                        v_state++;
                    }
                    break;
                case 14:
                    robot.drive_inches(6,v_useGyro);
                    v_state++;
                    break;
                case 15:
                    if (robot.drive_inches_complete())
                    {
                        v_state++;
                    }
                    break;
                case 16:
                    robot.blockgrabber_open();
                    v_state++;
                    break;
                case 17:
                    if(robot.timewait_Complete()){
                        v_state++;
                    }
                    break;
                case 18:
                    robot.drive_inches(-2,v_useGyro);
                    v_state++;
                    break;
                case 19:
                    if(robot.drive_inches_complete()){
                        v_state++;
                    }
                    break;
                case 20:
                    robot.set_message("the robot worked thank connor ");
                    //robot.play_jingle_bells();
                    v_state++;
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