#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>

#include "control/ControlFrame.h"
#include "control/CtrlComponents.h"
#include "interface/KeyBoard.h"

#ifdef COMPILE_WITH_REAL_ROBOT
#include "interface/IOSDK.h"
#endif // COMPILE_WITH_REAL_ROBOT

#ifdef COMPILE_WITH_ROS
#include "interface/IOROS.h"
#endif // COMPILE_WITH_ROS

bool running = true;

// over watch the ctrl+c command
void ShutDown(int sig)
{
    std::cout << "stop the controller" << std::endl;
    running = false;
}

void setProcessScheduler()
{
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

int main(int argc, char **argv)
{
    /* set real-time process */
    setProcessScheduler();
    /* set the print format */
    std::cout << std::fixed << std::setprecision(3);

#ifdef RUN_ROS
    ros::init(argc, argv, "hyleg_gazebo_servo");
#endif // RUN_ROS

    IOInterface *ioInter;
    CtrlPlatform ctrlPlat;

#ifdef COMPILE_WITH_SIMULATION
    ioInter = new IOROS();
    ctrlPlat = CtrlPlatform::GAZEBO;
#endif // COMPILE_WITH_SIMULATION

#ifdef COMPILE_WITH_REAL_ROBOT
    ioInter = new IOSDK();
    ctrlPlat = CtrlPlatform::REALROBOT;
#endif // COMPILE_WITH_REAL_ROBOT

    CtrlComponents *ctrlComp = new CtrlComponents(ioInter);
    ctrlComp->ctrlPlatform = ctrlPlat;
    ctrlComp->dt = 0.002;   //run at 500Hz
    ctrlComp->running = &running;

#ifdef ROBOT_TYPE_HYLEG
    ctrlComp->robotModel = new HyRobot();
#endif

    ControlFrame ctrlFrame(ctrlComp);

    signal(SIGINT, ShutDown);

        while (running)
    {
        ctrlFrame.run();
    }
    
    delete ctrlComp;
    return 0;
}
