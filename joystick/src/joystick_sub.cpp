#include <rclcpp/rclcpp.hpp>
#include <bits/stdc++.h>
#include "mav_msgs/msg/joystick.hpp"
using namespace std;
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
 
#define XBOX_AXIS_VAL_UP        -32767
#define XBOX_AXIS_VAL_DOWN      32767
#define XBOX_AXIS_VAL_LEFT      -32767
#define XBOX_AXIS_VAL_RIGHT     32767
 
#define XBOX_AXIS_VAL_MIN       -32767
#define XBOX_AXIS_VAL_MAX       32767
#define XBOX_AXIS_VAL_MID       0x00

class Joystick_sub : public rclcpp::Node {
private:
    rclcpp::Subscription<mav_msgs::msg::Joystick>::SharedPtr subscription_;
	int _time;
	int _lx;
	int _ly;
	int _y;
	int _a;
public:
    Joystick_sub() : Node("joystick_sub"){
        subscription_ = this->create_subscription<mav_msgs::msg::Joystick>("joystick_input", 10,
			[this](const mav_msgs::msg::Joystick::UniquePtr msg) {
			/*printf("\rTime:%8d A:%d B:%d X:%d Y:%d LB:%d RB:%d start:%d back:%d home:%d LO:%d RO:%d XX:%-6d YY:%-6d LX:%-6d LY:%-6d RX:%-6d RY:%-6d LT:%-6d RT:%-6d\n",
                    msg->time, msg->a, msg->b, msg->x, msg->y, msg->lb, msg->rb, msg->start, msg->back, msg->home, msg->lo, msg->ro,
                    msg->xx, msg->yy, msg->lx, msg->ly, msg->rx, msg->ry, msg->lt, msg->rt);*/
				cout << "initialted succeed" << endl;
				_time = msg -> time;
				_lx = msg -> lx;
				_ly = msg -> ly;
				_y = msg -> y;
				_a = msg -> a;
				printf("\rTime:%-8d LX:%-6d LY:%-6d Y:%d A:%d",_time,_lx,_ly,_y,_a);
				/*std::cout << "time: " << _time << std::endl;
				std::cout << "lx: " << _lx << std::endl;
				std::cout << "ly: " << _ly << std::endl;
				std::cout << "a: " << _a << std::endl;
				std::cout << "y: " << _y << std::endl;*/
			});
    }
};

int main(int argc, char *argv[])
{
	std::cout << "Starting joystick_node listener node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	std::cout << "1" << std::endl;
	rclcpp::spin(std::make_shared<Joystick_sub>());

	rclcpp::shutdown();
	return 0;
}


