#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/input.h>
#include <linux/joystick.h>
#include <rclcpp/rclcpp.hpp>
#include "mav_msgs/msg/joystick.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace mav_msgs::msg;

#define XBOX_TYPE_BUTTON    0x01
#define XBOX_TYPE_AXIS      0x02
 
#define XBOX_BUTTON_A       0x00
#define XBOX_BUTTON_B       0x01
#define XBOX_BUTTON_X       0x02
#define XBOX_BUTTON_Y       0x03
#define XBOX_BUTTON_LB      0x04
#define XBOX_BUTTON_RB      0x05
#define XBOX_BUTTON_START   0x06
#define XBOX_BUTTON_BACK    0x07
#define XBOX_BUTTON_HOME    0x08
#define XBOX_BUTTON_LO      0x09    /* 左摇杆按键 */
#define XBOX_BUTTON_RO      0x0a    /* 右摇杆按键 */
 
#define XBOX_BUTTON_ON      0x01
#define XBOX_BUTTON_OFF     0x00
 
#define XBOX_AXIS_LX        0x00    /* 左摇杆X轴 */
#define XBOX_AXIS_LY        0x01    /* 左摇杆Y轴 */
#define XBOX_AXIS_RX        0x03    /* 右摇杆X轴 */
#define XBOX_AXIS_RY        0x04    /* 右摇杆Y轴 */
#define XBOX_AXIS_LT        0x02
#define XBOX_AXIS_RT        0x05
#define XBOX_AXIS_XX        0x06    /* 方向键X轴 */
#define XBOX_AXIS_YY        0x07    /* 方向键Y轴 */
 
#define XBOX_AXIS_VAL_UP        -32767.0
#define XBOX_AXIS_VAL_DOWN      32767.0
#define XBOX_AXIS_VAL_LEFT      -32767.0
#define XBOX_AXIS_VAL_RIGHT     32767.0
 
#define XBOX_AXIS_VAL_MIN       -32767
#define XBOX_AXIS_VAL_MAX       32767
#define XBOX_AXIS_VAL_MID       0x00
 
typedef struct xbox_map
{
    int     time;
    int     a;
    int     b;
    int     x;
    int     y;
    int     lb;
    int     rb;
    int     start;
    int     back;
    int     home;
    int     lo;
    int     ro;
    int     lx;
    int     ly;
    int     rx;
    int     ry;
    int     lt;
    int     rt;
    int     xx;
    int     yy;
 
}xbox_map_t;

class Joystick_node : public rclcpp::Node{
public:
    Joystick_node() : Node("joystick"){

        joystick_input_publisher_ = this->create_publisher<mav_msgs::msg::Joystick>("/sc/joystick_input", 10);

        memset(&_map, 0, sizeof(xbox_map_t));

        xbox_fd = xbox_open("/dev/input/js1");
        if(xbox_fd < 0)
        {
            return;
        }

        auto timer_callback = [this]() -> void {
            ssize_t bytes = xbox_map_read(xbox_fd, &_map);
            printf("\rTime:%8d A:%d B:%d X:%d Y:%d LB:%d RB:%d start:%d back:%d home:%d LO:%d RO:%d XX:%-6d YY:%-6d LX:%-6d LY:%-6d RX:%-6d RY:%-6d LT:%-6d RT:%-6d",
                    _map.time, _map.a, _map.b, _map.x, _map.y, _map.lb, _map.rb, _map.start, _map.back, _map.home, _map.lo, _map.ro,
                    _map.xx, _map.yy, _map.lx, _map.ly, _map.rx, _map.ry, _map.lt, _map.rt);
            joystick_input_publish();
            std::cout << "joystick input sended..." << std::endl;
		};
        //auto new_timer_callback = std::bind(&Joystick_node::timer_callback, this, xbox_fd);
        timer_ = this->create_wall_timer(1ms, timer_callback);
        /*while(1){
            len = xbox_map_read(xbox_fd, &_map);
            if (len < 0)
            {
                usleep(10*1000);
                continue;
            }

            printf("\rTime:%8d A:%d B:%d X:%d Y:%d LB:%d RB:%d start:%d back:%d home:%d LO:%d RO:%d XX:%-6d YY:%-6d LX:%-6d LY:%-6d RX:%-6d RY:%-6d LT:%-6d RT:%-6d",
                    _map.time, _map.a, _map.b, _map.x, _map.y, _map.lb, _map.rb, _map.start, _map.back, _map.home, _map.lo, _map.ro,
                    _map.xx, _map.yy, _map.lx, _map.ly, _map.rx, _map.ry, _map.lt, _map.rt);
            fflush(stdout);
        }*/
    }
private:
    rclcpp::Publisher<mav_msgs::msg::Joystick>::SharedPtr joystick_input_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
    xbox_map_t _map;
    int xbox_fd ;
    int len, type;
    int axis_value, button_value;
    int number_of_axis, number_of_buttons;

    void joystick_input_publish();
    int xbox_map_read(int xbox_fd, xbox_map_t *map);
    void xbox_close(int xbox_fd);
    int xbox_open(char *file_name);
}; 

/**
 * @brief Publish normalized joystick input [-1,1] 
 * 
 */

void Joystick_node::joystick_input_publish(){
    mav_msgs::msg::Joystick msg;
    msg.time = _map.time;
    msg.a = _map.a;
    //msg.b = 3；//_map.b;
    //msg.x = 4；//_map.x;
    msg.y = _map.y;
    //msg.lb = 6；//_map.lb;
    //msg.rb = 7；//_map.rb;
    //msg.start = 8；//_map.start;
    //msg.back = 9；//_map.back;
    //msg.home = 10；//_map.home;
    //msg.lo = 11；//_map.lo;
    //msg.ro = 12；//_map.ro;
    //msg.xx = 13；//_map.xx;
    //msg.yy = 14；//_map.yy;
    msg.lx = _map.lx / XBOX_AXIS_VAL_LEFT;
    msg.ly = _map.ly / XBOX_AXIS_VAL_RIGHT;
    //msg.rx = 17；//_map.rx;
    //msg.ry = 18；//_map.ry;
    //msg.lt = 19；//_map.lt;
    //msg.rt = 20；//_map.rt;*/
    joystick_input_publisher_ -> publish(msg);
}

/**
 * @brief receive XBOX 360 joystick input
 * 
 * @param xbox_fd 
 * @param map 
 * @return int 
 */
int Joystick_node::xbox_map_read(int xbox_fd, xbox_map_t *map){
    int len, type, number, value;//定义js_event局部变量
    struct js_event js;

    len = read(xbox_fd, &js, sizeof(struct js_event));
    if (len < 0)
    {
        perror("read");
        return -1;
    }
    type = js.type;
    number = js.number;
    value = js.value;
    map->time = js.time;

    if (type == JS_EVENT_BUTTON){
        switch (number){
            case XBOX_BUTTON_A:
                map->a = value;
                break;

            case XBOX_BUTTON_B:
                map->b = value;
                break;

            case XBOX_BUTTON_X:
                map->x = value;
                break;

            case XBOX_BUTTON_Y:
                map->y = value;
                break;

            case XBOX_BUTTON_LB:
                map->lb = value;
                break;

            case XBOX_BUTTON_RB:
                map->rb = value;
                break;

            case XBOX_BUTTON_START:
                map->start = value;
                break;

            case XBOX_BUTTON_BACK:
                map->back = value;
                break;

            case XBOX_BUTTON_HOME:
                map->home = value;
                break;

            case XBOX_BUTTON_LO:
                map->lo = value;
                break;

            case XBOX_BUTTON_RO:
                map->ro = value;
                break;

            default:
                break;
        }
    }
    else if (type == JS_EVENT_AXIS){
        switch(number){
            case XBOX_AXIS_LX:
                map->lx = value;
                break;

            case XBOX_AXIS_LY:
                map->ly = value;
                break;

            case XBOX_AXIS_RX:
                map->rx = value;
                break;

            case XBOX_AXIS_RY:
                map->ry = value;
                break;

            case XBOX_AXIS_LT:
                map->lt = value;
                break;

            case XBOX_AXIS_RT:
                map->rt = value;
                break;

            case XBOX_AXIS_XX:
                map->xx = value;
                break;

            case XBOX_AXIS_YY:
                map->yy = value;
                break;

            default:
                break;
        }
    }
    else{
        /* Init do nothing */
    }
    return len;
}
int Joystick_node::xbox_open(char *file_name){
    int xbox_fd;
    xbox_fd = open(file_name, O_RDONLY | O_NONBLOCK);
    if (xbox_fd < 0)
    {
        perror("open");
        return -1;
    }
    return xbox_fd;
}
void Joystick_node::xbox_close(int xbox_fd) {   
    close(xbox_fd);
    return;
}


int main(int argc, char* argv[]) {
	std::cout << "Starting joystick input node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Joystick_node>());

	rclcpp::shutdown();
	return 0;
}