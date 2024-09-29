#include <QThread>
#include <QDebug>
#include <ur_rtde/rtde_receive_interface.h>

// Define a worker thread class that inherits from QThread
class RobotThread : public QThread
{
    Q_OBJECT

public:
    explicit RobotThread(QObject *parent = nullptr);

    ~RobotThread() override;


    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    std::vector<double> tcp_pose_;

protected:
    // Override the run() method to implement your thread's functionality
    void run() override;


private:

    ur_rtde::RTDEReceiveInterface* rtde_receive_; // RTDE interface instance

};
