#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class desvio : public rclcpp::Node
{
private:

    float virar = 0;

    float dados_salvos_esq_;

    float dados_salvos_dir_;

    void mov()
    {
        auto message = geometry_msgs::msg::Twist();

        if (dados_salvos_dir_ < 0.4 || dados_salvos_esq_ < 0.4)
        {
            message.linear.x = 0.1;
        
            if(dados_salvos_dir_ < dados_salvos_esq_) {
                virar =  -1;

                message.angular.z = virar;
            } else {
                virar = 1;

                message.angular.z = virar;
            }
        }
        else
        {

            message.linear.x = 0.4;

            message.angular.z = 0;
        }

        publisher_->publish(message);
    }

    void dados_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int total = msg->ranges.size();

        int largura = 30;

        float pertoEsq = 10;

        float pertoDir = 10;

        for (int i = 0; i <= largura; i++)
        {
            if (msg->ranges[i] < pertoDir && msg->ranges[i])
            {
                if (msg->ranges[i] < pertoDir && msg->ranges[i] > msg->range_min)
                {
                    pertoDir = msg->ranges[i];
                }
            }
        }

        for (int i = total - 1; i >= (total - largura); i--)
        {
            if (msg->ranges[i] < pertoEsq && msg->ranges[i])
            {
                if (msg->ranges[i] < pertoEsq && msg->ranges[i] > msg->range_min)
                {
                    pertoEsq = msg->ranges[i];
                }
            }
        }

        dados_salvos_esq_ = pertoEsq;
        dados_salvos_dir_ = pertoDir;
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    desvio() : Node("no_movimento")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&desvio::dados_laser, this, _1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&desvio::mov, this));
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<desvio>());
    rclcpp::shutdown();
    return 0;
}