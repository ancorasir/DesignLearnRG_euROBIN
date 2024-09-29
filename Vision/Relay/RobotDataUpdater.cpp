#include "RobotDataUpdater.h"

RobotThread::RobotThread(QObject *parent)
    : QThread(parent)
{
    // Initialize RTDE Receive Interface
    rtde_receive_ = new ur_rtde::RTDEReceiveInterface("192.168.1.102");

    if (!rtde_receive_->isConnected()) {
        qDebug() << "Failed to connect to RTDE interface!";
        // Handle connection failure
        delete rtde_receive_;
        rtde_receive_ = nullptr;
    }
}

RobotThread::~RobotThread()
{
    if(rtde_receive_)
    {
        rtde_receive_->disconnect();
        delete rtde_receive_;
    }
}

void RobotThread::run()
{
    if(!rtde_receive_)
        return;

    // Perform continuous operations in the thread
    while (!isInterruptionRequested()) {
        // Example: Read data from RTDE interface
        joint_positions_ = rtde_receive_->getActualQ();
        tcp_pose_ = rtde_receive_->getActualTCPPose();
        joint_velocities_ = rtde_receive_->getActualQd();

        // // Process received data as needed
        // qDebug() << "Received joint positions:" << joint_positions[0] << joint_positions[1] << joint_positions[2];
        // qDebug() << "Received TCP pose:" << tcp_pose[0] << tcp_pose[1] << tcp_pose[2];

        // Sleep or perform other operations
        msleep(100); // Example: wait for 100 milliseconds
    }
}
