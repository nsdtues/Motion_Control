//system_node.cpp  
//������ 2018��5��6��
//������ 2018��5��6��

#include "motion_control/motion_control.h"
#include "boost/thread.hpp"
#include <ros/ros.h>
#include "motion_control/msg_motion_cmd.h"
#include "motion_control/sys_cmd_msg_to_motor.h"
#include "motion_control/node_motor_msg_to_sys.h"
#include "motion_control/msg_motion_evt.h"
#include "predefinition.h"

sem_t sem_sub;
sem_t sem_pub;

ros::Publisher pub_msg_motion_cmd;
ros::Publisher pub_node_motor_msg_to_sys;

motion_control::msg_motion_evt msg_motion_evt;

std::string motion_state;
uint32_t motor_check_results;

void get_default_settings(void)
{
    return;
}

void sys_cmd_msg_to_motor_callback(const motion_control::sys_cmd_msg_to_motor& cmd_input)
{
	motion_state = cmd_input.syscmd;
	sem_post(&sem_sub);	
}

void msg_motion_evt_callback(const motion_control::msg_motion_evt& evt_input)
{
	motor_check_results = evt_input.check_results;
	sem_post(&sem_pub);		
}


void motion_cmd_pub_loop(void)
{
	struct timespec ts;
    uint32_t timeout = 1000;
	
	motion_control::msg_motion_cmd msg_motion_cmd;
	motion_control::node_motor_msg_to_sys node_motor_msg_to_sys;	
	
#if(((RUN_MOTION == REAL)&&(WHERE_MOTION == EXOSUIT_VERSION))||(SYSTEM_CLIENT_TEST == SYSTEM_ON))
    get_default_settings();
    msg_motion_cmd.forceaid = 3;
#endif

#if(((RUN_MOTION == DEBUG)||(WHERE_MOTION == DESKTOP_VERSION))&&(SYSTEM_CLIENT_TEST == SYSTEM_OFF))
    msg_motion_cmd.state = CTL_CMDINITIAL;
    get_default_settings();
	msg_motion_cmd.forceaid = 3;

	pub_msg_motion_cmd.publish(msg_motion_cmd);
	
    sem_wait(&sem_sub);

    sleep(2);

    if(msg_motion_evt.check_results == module_check_success){
        msg_motion_cmd.state = CTL_CMDMOTIONSTART;
        node_motor_msg_to_sys.evt = "initialsuccess";
    }else{
        node_motor_msg_to_sys.evt = "initialerror";
    }

#endif	
	
	for(;;){
		sem_wait(&sem_sub);
		if(motion_state == "cmdmotorintial"){
			if(msg_motion_cmd.state != CTL_CMDINITIAL){
				msg_motion_cmd.state = CTL_CMDINITIAL;
				
				timeout = 120;
				ts = gettimeout(timeout);
				
				pub_msg_motion_cmd.publish(msg_motion_cmd);
				if(sem_timedwait(&sem_pub,&ts) == 0){
					if(msg_motion_evt.check_results == module_check_success){
						node_motor_msg_to_sys.evt = "initialsuccess";
					}else{
						node_motor_msg_to_sys.evt = "initialerror";
					}
				}else{
					node_motor_msg_to_sys.evt = "initialerror";
				}	
			}else{
				node_motor_msg_to_sys.evt = "initialsuccess";
			}
			pub_node_motor_msg_to_sys.publish(node_motor_msg_to_sys);
		}else if(motion_state == "cmdmotorshutdown"){
            if(msg_motion_cmd.state != CTL_CMDPOWERDOWN){
                msg_motion_cmd.state = CTL_CMDPOWERDOWN;

                timeout = 10;
                ts = gettimeout(timeout);

				pub_msg_motion_cmd.publish(msg_motion_cmd);
                if(sem_timedwait(&sem_pub,&ts)==0){
					node_motor_msg_to_sys.evt = "shutdownsuccess";
                }else{
					node_motor_msg_to_sys.evt = "shutdownerror";
                }
            } else{
                node_motor_msg_to_sys.evt = "shutdownsuccess";
            }
            pub_node_motor_msg_to_sys.publish(node_motor_msg_to_sys);
		}else if(motion_state == "cmdmotorstop"){
			if(msg_motion_cmd.state != CTL_CMDMOTIONSTOP){
                msg_motion_cmd.state = CTL_CMDMOTIONSTOP;
				
                timeout = 10;
                ts = gettimeout(timeout);

				pub_msg_motion_cmd.publish(msg_motion_cmd);
                if(sem_timedwait(&sem_pub,&ts) == 0){
					node_motor_msg_to_sys.evt = "stopsuccess";
                }else{
					node_motor_msg_to_sys.evt = "stoperrorID";
                }
            } else{
                node_motor_msg_to_sys.evt = "stopsuccess";
            }
			pub_node_motor_msg_to_sys.publish(node_motor_msg_to_sys);
		}else if(motion_state == "cmdmotorpause"){
			if(msg_motion_cmd.state != CTL_CMDMOTIONSLEEP){
                msg_motion_cmd.state = CTL_CMDMOTIONSLEEP;

                timeout = 10;
                ts = gettimeout(timeout);

                pub_msg_motion_cmd.publish(msg_motion_cmd);

                if(sem_timedwait(&sem_pub,&ts) == 0){
					node_motor_msg_to_sys.evt = "pausesuccess";
                }else{
					node_motor_msg_to_sys.evt = "pausesuccess";
                }
            }else{
                node_motor_msg_to_sys.evt = "pausesuccess";
            }
			
            pub_node_motor_msg_to_sys.publish(node_motor_msg_to_sys);
		}else if(motion_state == "cmdmotorstart"){
			if(msg_motion_cmd.state != CTL_CMDMOTIONSTART){
                msg_motion_cmd.state = CTL_CMDMOTIONSTART;

                timeout = 10;
                ts = gettimeout(timeout);

                pub_msg_motion_cmd.publish(msg_motion_cmd);

                if(sem_timedwait(&sem_pub,&ts) == 0){
					node_motor_msg_to_sys.evt = "motorstartsuccess";
                }else{
					node_motor_msg_to_sys.evt = "starterrorID";
                }
            }else{
				node_motor_msg_to_sys.evt = "motorstartsuccess";
            }

			pub_node_motor_msg_to_sys.publish(node_motor_msg_to_sys);
		}else if(motion_state == "cmdmotorsetparam"){
			node_motor_msg_to_sys.evt = "setparamsuccess";
            pub_node_motor_msg_to_sys.publish(node_motor_msg_to_sys);

		}else if(motion_state == "cmdmotorforceaid"){
			std::string::size_type pos = motion_state.find("cmdmotorforceaid");
			if(pos == std::string::npos){

			}else{
				std::stringstream stream;
				motion_state.erase(0,sizeof("cmdmotorforceaid:")-1);
				stream << motion_state;
				stream >> msg_motion_cmd.forceaid;
				pub_msg_motion_cmd.publish(msg_motion_cmd);
			}
            node_motor_msg_to_sys.evt = "setforceaidsuccess";
            pub_node_motor_msg_to_sys.publish(node_motor_msg_to_sys);		
		}else if(motion_state == "cmdmotormode"){
			if(motion_state == "cmdmotormode"){
				msg_motion_cmd.mode = MOTION_MODE_GAIT;
			}else if(motion_state == "cmdmotormode"){
				msg_motion_cmd.mode = MOTION_MODE_FIXED;
			}else if(motion_state == "cmdmotormode"){
				msg_motion_cmd.mode = MOTION_MODE_RELAX;
			}			
			pub_msg_motion_cmd.publish(msg_motion_cmd);
			node_motor_msg_to_sys.evt = "setmodesuccess";
			pub_node_motor_msg_to_sys.publish(node_motor_msg_to_sys);		
		}else{

        }
	}
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "system_node");
    ros::NodeHandle nh;
	
	pub_msg_motion_cmd = nh.advertise<motion_control::msg_motion_cmd>("msg_motion_cmd",50,true);
	pub_node_motor_msg_to_sys = nh.advertise<motion_control::msg_motion_cmd>("node_motor_msg_to_sys",50,true);
	ros::Subscriber sub_sys_cmd_msg_to_motor = nh.subscribe("sys_cmd_msg_to_motor", 50, sys_cmd_msg_to_motor_callback);
	ros::Subscriber msg_motion_evt = nh.subscribe("msg_motion_evt", 50, msg_motion_evt_callback);
	boost::thread motion_cmd_pub(&motion_cmd_pub_loop);
	
	
	ros::spin();
    return 0;	
}