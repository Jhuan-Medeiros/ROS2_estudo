#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" // biblioteca pra mandar mensagens de movimento
#include "sensor_msgs/msg/laser_scan.hpp" // biblioteca pra mandar informacao do lidar

using namespace std::chrono_literals; // habilita o uso de tempos como hr s ms
using std::placeholders::_1; // argumento para o bind

class desvio : public rclcpp::Node // cria um nó publico
{
private:

    float virar = 0; // variavel pra mudar a direcao

    float dados_salvos_esq_; // variavel que guarda os dados que sao lidos no lidar

    float dados_salvos_dir_;

    void mov()
    {
        auto message = geometry_msgs::msg::Twist(); // salva como message o topico de movimentacao

        if (dados_salvos_dir_ < 0.4 || dados_salvos_esq_ < 0.4) // checa se esta perto de algo
        {
            message.linear.x = 0.1; // desacelera
        
            if(dados_salvos_dir_ < dados_salvos_esq_) { // se tem algo perto da esquerda 
                virar =  -1;

                message.angular.z = virar;
            } else { // se tem algo perto da direita
                virar = 1;

                message.angular.z = virar;
            }
        }
        else // se nn esta prto de nada anda rápido
        {
    
            message.linear.x = 0.4;

            message.angular.z = 0;
        }

        publisher_->publish(message); // envia no tópico de movimento os dados
    }

    void dados_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg) // sistema de lidar
    {
        int total = msg->ranges.size(); // pega toda a circunferencia do radar

        int largura = 30; // largura que o radar le em graus

        float pertoEsq = 10; // distancia padrao de seguranca pra funcionamento do lidar

        float pertoDir = 10;

        for (int i = 0; i <= largura; i++) // checa se o grau de visao da direita
        {
            if (msg->ranges[i] < pertoDir && msg->ranges[i])
            {
                if (msg->ranges[i] < pertoDir && msg->ranges[i] > msg->range_min)
                {
                    pertoDir = msg->ranges[i]; // se ter algo detectado salva o valor
                }
            }
        }

        for (int i = total - 1; i >= (total - largura); i--) // checa se o grau de visao da esquerda
        {
            if (msg->ranges[i] < pertoEsq && msg->ranges[i])
            {
                if (msg->ranges[i] < pertoEsq && msg->ranges[i] > msg->range_min)
                {
                    pertoEsq = msg->ranges[i]; // se ter algo detectado salva o valor
                }
            }
        }

        dados_salvos_esq_ = pertoEsq; // transforma os valores salvos em valores globais
        dados_salvos_dir_ = pertoDir;
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_; // cria a inscricao no topico do lidar
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; // cria como editor no topico do movimento
    rclcpp::TimerBase::SharedPtr timer_; // cria um timer pra poder realizar funcionamentos assincronos

public:
    desvio() : Node("no_movimento") // cria o no chamado no_movimento
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&desvio::dados_laser, this, _1)); // faz a leitura do topico de lidar
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); // manda os dados no topico de movimento
        timer_ = this->create_wall_timer(100ms, std::bind(&desvio::mov, this)); // cria o timer que serve como millis
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<desvio>());
    rclcpp::shutdown();
    return 0;
}
