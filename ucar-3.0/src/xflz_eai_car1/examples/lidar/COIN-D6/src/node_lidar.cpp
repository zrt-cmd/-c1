#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <thread>
#include <limits>

#include "node_lidar.h"
#include "msg_recept.h"
#include "serial_port.h"
#include "lidar_data_processing.h"
#include "point_cloud_optimize.h"
#include "lidar_information.h"
#include "mtime.h"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16.hpp"

//#include "interface/Error.hpp"
using namespace std;

node_lidar_t node_lidar;

//#define ENCRYPTION_T

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("m1ct_d2")
    {
      subscription_ = this->create_subscription<std_msgs::msg::UInt16>(
      "lidar_status", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::UInt16::SharedPtr msg) const
    {
		switch (msg->data)
		{
			/*启动雷达*/
			case 1:
				node_lidar.lidar_status.lidar_ready = true;
				node_lidar.lidar_status.lidar_abnormal_state = 0;
				printf("#start lidar\n");
				break;
			/*关闭雷达*/
			case 2:
				node_lidar.lidar_status.lidar_ready = false;
				node_lidar.lidar_status.close_lidar = true;
				node_lidar.serial_port->write_data(end_lidar,4);
				printf("#stop lidar\n");
				break;
			/*high_exposure*/
			case 3:
				node_lidar.serial_port->write_data(high_exposure,4);
				break;
			/*low_exposure*/
			case 4:
				node_lidar.serial_port->write_data(low_exposure,4);
				break;
			/*清空异常状态*/
			case 5:
				node_lidar.lidar_status.lidar_abnormal_state = 0;
				break;
			/*高转速*/
			case 6:
				node_lidar.serial_port->write_data(high_speed,4);
				node_lidar.lidar_general_info.frequency_max = 103;
				node_lidar.lidar_general_info.frequency_min = 97;
				break;
			/*低转速*/
			case 7:
				node_lidar.serial_port->write_data(low_speed,4);
				node_lidar.lidar_general_info.frequency_max = 68;
				node_lidar.lidar_general_info.frequency_min = 52;
				break;
			
			default:
				break;
		}
     // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr subscription_;
};

node_lidar_t::~node_lidar_t(){
  	if(scan_node_buf)
	{
		delete[] scan_node_buf;
		scan_node_buf = NULL;
	}
	if (globalRecvBuffer)
	{
		delete[] globalRecvBuffer;
		globalRecvBuffer = NULL;
	}
	printf("关闭雷达");
	node_lidar.serial_port->close();
	node_lidar.lidar_status.lidar_ready = false;
	node_lidar.lidar_status.close_lidar = true;
	flushSerial();

}

/*清空缓存区数据*/
void flushSerial()
{
	if (!node_lidar.lidar_status.isConnected){
		return;
	}
	
	size_t len = node_lidar.serial_port->available();
	
	if (len)
	{
		uint8_t *buffer = static_cast<uint8_t *>(alloca(len * sizeof(uint8_t)));
		size_t bytes_read = node_lidar.serial_port->read_data(buffer, len);
	}

	sleep_ms(20);
}

/*激光雷达启动状态判断函数*/
bool lidar_state_judgment()
{
	static bool status_judge = false;     //整体状态判断
	static bool lidar_flush = false;      //是否已经下发雷达启动指令
	static bool wait_speed_right = false; //是否获取到调速信息
	static bool lidar_start_flag = false; //下发雷达启动指令后的雷达反馈标志

	static uint64_t lidar_status_time = 0; //收到启动指令或者重启指令的时间


	if(node_lidar.lidar_status.lidar_ready != node_lidar.lidar_status.lidar_last_status || node_lidar.lidar_status.close_lidar)
	{
		printf("状态切换\n");

		node_lidar.lidar_status.close_lidar = false;
		node_lidar.lidar_status.lidar_last_status = node_lidar.lidar_status.lidar_ready;

		lidar_flush = false;
		wait_speed_right = false;
		lidar_start_flag = false;

		lidar_status_time = current_times();
		flushSerial();
	}
	if(node_lidar.lidar_status.lidar_trap_restart)
	{
		printf("状态异常重新启动 %lld\n",lidar_status_time);
		
		node_lidar.lidar_status.lidar_trap_restart = false;

		
		wait_speed_right = false;
		lidar_flush = false;
		lidar_start_flag = false;

		lidar_status_time = current_times();
		node_lidar.serial_port->write_data(end_lidar,4);
	}
	if(node_lidar.lidar_status.lidar_ready && !wait_speed_right)
	{
		
		if(current_times() - lidar_status_time > 1000 && !lidar_flush)
		{
			switch (node_lidar.lidar_general_info.version)
			{
				case M1C1_Mini_v1:
					printf("V1版本启动雷达\n");
					node_lidar.serial_port->write_data(start_lidar,4);
					wait_speed_right = true;
					break;
				
				case M1C1_Mini_v2:
					printf("V2版本启动雷达\n");
					node_lidar.serial_port->write_data(start_lidar,4);
					wait_speed_right = true;
					break;
				
				case M1CT_Coin_D2:
					printf("D2版本启动雷达\n");
					node_lidar.serial_port->write_data(start_lidar,4);
					wait_speed_right = true;
					break;
				
				default:
					break;
			}
		}
		node_lidar.lidar_time.lidar_frequence_abnormal_time = current_times();
		node_lidar.lidar_time.system_start_time = current_times();
	}
	return wait_speed_right;
}

/************************************************************************/
/*  激光数据解析线程　Laser data analysis thread                           */
/************************************************************************/
int read_forever()
{	
	node_info  local_buf[128];
	size_t     count = 128;
	node_info  local_scan[1000];
	size_t     scan_count = 0;
	result_t   ans = RESULT_FAIL;
	
	memset(local_scan, 0, sizeof(local_scan));

	node_lidar.lidar_time.scan_time_record = current_times();

	while (1)
	{
		bool state_jugde = lidar_state_judgment();
		if(state_jugde)
		{
			count = 128;
			ans = node_lidar.lidar_data_processing.waitScanData(local_buf, count);
			if(!IS_OK(ans))
			{
				if(current_times()-node_lidar.lidar_time.system_start_time > 3000 )
				{
					if(!node_lidar.lidar_status.lidar_restart_try)
					{
						printf("尝试重启雷达\n");
						node_lidar.lidar_status.lidar_restart_try = true;
						node_lidar.lidar_status.lidar_trap_restart = true;
					}else{
						printf("@@@雷达被卡住\n");
						node_lidar.lidar_status.lidar_abnormal_state |= 0x01;
						usleep(100);
					}
				}	
			}else{
				node_lidar.lidar_status.lidar_restart_try = false;
				node_lidar.lidar_time.system_start_time = current_times();
			}
			for (size_t pos = 0; pos < count; ++pos)
			{
				if (local_buf[pos].sync_flag & LIDAR_RESP_MEASUREMENT_SYNCBIT)
				{
					if ((local_scan[0].sync_flag & LIDAR_RESP_MEASUREMENT_SYNCBIT))
					{
						local_scan[0].stamp = local_buf[pos].stamp;
						local_scan[0].scan_frequence = local_buf[pos].scan_frequence;
		
						/*频率异常超过30秒，触发异常状态*/
						if(local_scan[0].scan_frequence > node_lidar.lidar_general_info.frequency_max 
						    || local_scan[0].scan_frequence < node_lidar.lidar_general_info.frequency_min)
						{
							if(current_times()-node_lidar.lidar_time.lidar_frequence_abnormal_time > 30000)
							{
								node_lidar.lidar_status.lidar_abnormal_state |= 0x02;
							}
						}else{
							node_lidar.lidar_time.lidar_frequence_abnormal_time = current_times();
						}
						
						
						node_lidar._lock.lock();
						if((node_lidar.lidar_time.scan_time_current - node_lidar.lidar_time.scan_time_record) > 2000)
						{
							// printf("full----- count=%d,time=%lld\n",scan_count,current_times());
							node_lidar.lidar_time.scan_time_record = node_lidar.lidar_time.scan_time_current;
						}
						node_lidar.lidar_time.scan_start_time = node_lidar.lidar_time.tim_scan_start;
						node_lidar.lidar_time.scan_end_time = node_lidar.lidar_time.tim_scan_end;
						if(node_lidar.lidar_time.tim_scan_start != node_lidar.lidar_time.tim_scan_end)
						{
							node_lidar.lidar_time.tim_scan_start = node_lidar.lidar_time.tim_scan_end;
						}

						memcpy(node_lidar.scan_node_buf, local_scan, scan_count * sizeof(node_info));
						node_lidar.scan_node_count = scan_count;
						node_lidar.lidar_time.scan_time_current = current_times();
						node_lidar._dataEvent.set();
						node_lidar._lock.unlock();
					}
					scan_count = 0;
				}
				local_scan[scan_count++] = local_buf[pos];
				if (scan_count == _countof(local_scan))
				{
					scan_count -= 1;
				}
			}
		}
		else{
			flushSerial();
			delay(100);
		}
		
	}
	return RESULT_OK;
}

/*线程事件同步函数*/
result_t grabScanData(uint32_t timeout) {
	switch (node_lidar._dataEvent.wait(timeout)) {
		case Event::EVENT_TIMEOUT:
			return RESULT_TIMEOUT;

		case Event::EVENT_OK: 
			{
				node_lidar._lock.lock();
				if (node_lidar.scan_node_count == 0) {
					return RESULT_FAIL;
				}
				node_lidar._lock.unlock();
			}
			return RESULT_OK;

		default:
			return RESULT_FAIL;
	}
}

/*处理雷达的线程*/
bool data_handling(LaserScan &outscan)
{

	//node_lidar.lidar_time.tim_scan_start = getTime();	
	if(grabScanData(2000)==RESULT_OK)
	{
		send_lidar_data(outscan);
		return true;
	}else{
		return false;
	}
	
}


/*处理最新一圈雷达的数据*/
int send_lidar_data(LaserScan &outscan)
{
	node_lidar._lock.lock();

	size_t count = node_lidar.scan_node_count;

	if(count >= MAX_SCAN_NODES || count == 0)
	{
		node_lidar._lock.unlock();
		return 0;
	}

	uint64_t scan_time = (node_lidar.lidar_time.scan_end_time - node_lidar.lidar_time.scan_start_time);

	node_lidar.lidar_block.lidar_zero_count = 0;

	size_t actual_points = count - 1;
	outscan.config.angle_increment = (2.0*M_PI/count);
	outscan.config.min_angle = 0;
	outscan.config.max_angle = outscan.config.angle_increment * (actual_points - 1);
	outscan.config.min_range = 0.05;
	outscan.config.max_range = 12.0;
	outscan.config.scan_time =  static_cast<float>(scan_time * 1.0 / 1e9);
	outscan.config.time_increment = outscan.config.scan_time / (double)(count - 1);
	outscan.stamp = node_lidar.lidar_time.scan_start_time;

	float range = 0;
	float angle = 0.0;
	uint16_t intensity = 0;
	for (int i = count-1; i > 0; i--)
	{
		LaserPoint point;
		angle = static_cast<float>((node_lidar.scan_node_buf[count -i].angle_q6_checkbit >>
									LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) /64.0f);
		range = (node_lidar.scan_node_buf[i].distance_q2/1000.0f);
		intensity = node_lidar.scan_node_buf[i].sync_quality;

		if(node_lidar.scan_node_buf[i].exp_m == 1)
		{
			intensity = 255;
		}else{
			if(intensity >= 255)
			{
				intensity = 254;
			}
		}

		if(0 <= angle && angle <= 360){
			point.range = range;
			point.angle = angle;
			point.intensity = intensity;
		}else{
			point.range = 0.0;
			point.intensity = 0;
			point.angle = 0.0;
		}

		if(range < 0.1)
		{
			node_lidar.lidar_block.lidar_zero_count++;
		}
		outscan.points.push_back(point);
	}

	node_lidar._lock.unlock();

	node_lidar.optimize_lidar.lidar_blocked_judge(count);

	if (node_lidar.lidar_status.FilterEnable)
	{
		node_lidar.optimize_lidar.PointCloudFilter(&outscan);
	}

	return 0;
}

/*设置串口信息的函数*/
bool lidar_set_port()
{
	if (node_lidar.lidar_status.isConnected)
	{
		return true;
	}

	std::cout << "***" << node_lidar.lidar_general_info.port << std::endl;
	node_lidar.serial_port=make_shared<Serial_Port>(node_lidar.lidar_general_info.port,
							node_lidar.lidar_general_info.m_SerialBaudrate,Timeout::simpleTimeout(DEFAULT_TIMEOUT));
	if (!node_lidar.serial_port->open())
	{
		return false;
	}
	node_lidar.lidar_status.isConnected=true;
	sleep_ms(100);
    node_lidar.serial_port->setDTR(0);
	return true;
}

/*初始化函数*/
bool initialize()
{
	if(node_lidar.lidar_status.optimize_enable)
	{
		//求雷达安装位置到扫地机边缘的距离
		node_lidar.optimize_lidar.lidar_blocked_init();
	}

	switch (node_lidar.lidar_general_info.version)
	{
	case M1C1_Mini_v1:
		printf("version M1C1_Mini_v1\n");
		node_lidar.lidar_general_info.m_SerialBaudrate = 115200;
		node_lidar.lidar_data_processing.PackageSampleBytes = 2;
		break;

	case M1C1_Mini_v2:
		printf("version M1C1_Mini_v2\n");
		node_lidar.lidar_general_info.m_SerialBaudrate = 150000;
		node_lidar.lidar_data_processing.PackageSampleBytes = 3;
		node_lidar.lidar_general_info.m_intensities = true;
		break;

	case M1CT_Coin_Plus:
		printf("version M1CT_Coin_Plus\n");
		node_lidar.lidar_general_info.m_SerialBaudrate = 115200;
		node_lidar.lidar_data_processing.PackageSampleBytes = 3;
		break;
	
	case M1CT_Coin_D2:
		printf("version M1CT_D2\n");
		node_lidar.lidar_general_info.m_SerialBaudrate = 230400;
		node_lidar.lidar_data_processing.PackageSampleBytes = 3;
		node_lidar.lidar_general_info.m_intensities = true;
		break;
	
	default:
		break;
	}

	//设置通信串口
	if(!lidar_set_port()){
		printf("lidar_set_port wrong\n");
		return false;
	}

	//获取串口获取每个byte所用的时间
	node_lidar.lidar_general_info.trans_delay = node_lidar.serial_port->getByteTime();
	node_lidar.scan_node_buf = new node_info[1000];
	node_lidar.globalRecvBuffer = new uint8_t[sizeof(node_packages)];
	return true;
}

/*接收外部消息的回调函数*/
/*
void topic_callback(const std_msgs::msg::UInt16::SharedPtr msg)
{
	printf("topic back----\n");
	switch (msg->data)
	{
		/*启动雷达*/
		/*case 1:
			node_lidar.lidar_status.lidar_ready = true;
			printf("ready to start lidar------\n");
			break;*/
		/*关闭雷达*/
		/*case 2:
			node_lidar.lidar_status.lidar_ready = false;
			node_lidar.lidar_status.close_lidar = true;
			node_lidar.serial_port->write_data(end_lidar,4);
			break;*/
		/*异常信息的反馈*/
		/*case 3:
			node_lidar.lidar_status.lidar_abnormal_state = 0;
			break;*/
		/*
		default:
			break;
	}
}*/

int topic_thread()
{
	rclcpp::spin(std::make_shared<MinimalSubscriber>());
}

int node_start(int argc, char **argv)
{


	// rclcpp::init 已在 main() 中调用，此处不再重复
	auto node = rclcpp::Node::make_shared("c1ct_d2");
	
	node->declare_parameter("port", std::string("/dev/car1_lidar"));
  	node->get_parameter("port", node_lidar.lidar_general_info.port);

	node->declare_parameter("baudrate", 230400);
  	node->get_parameter("baudrate", node_lidar.lidar_general_info.m_SerialBaudrate);

	node->declare_parameter("frame_id", std::string("laser_link"));
  	node->get_parameter("frame_id", node_lidar.lidar_general_info.frame_id);

	node->declare_parameter("version", 4);
  	node->get_parameter("version", node_lidar.lidar_general_info.version);
	//std::string frame_id = "base_scan";
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_pub;
	error_pub = node->create_publisher<std_msgs::msg::String>("lsd_error", 10);
	std_msgs::msg::String pubdata;
	/*
	auto subscription_ = node->create_subscription<std_msgs::msg::UInt16>(
      "lidar_status", 10, topic_callback);*/
	
	
	bool ret_init = initialize();

	if(!ret_init){
		printf("node_lidar init error\n");
		pubdata.data="node_lidar init error\n";
		error_pub->publish(pubdata);
		while(rclcpp::ok())
		{
			error_pub->publish(pubdata);
			delay(3000);
		}
		
		return -1;
	}
	/*读取激光雷达数据的线程*/
	thread t1(read_forever);
	t1.detach();

	thread t2(topic_thread);
	t2.detach();

	/*处理雷达数据的线程*/
	/*
	thread t2(data_handling(node));
	t2.detach();*/
	
	node_lidar.lidar_status.lidar_ready = true;

	auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan",
		rclcpp::SensorDataQoS());

	while(rclcpp::ok())
	{
		rclcpp::spin_some(node);
		if(node_lidar.lidar_status.lidar_abnormal_state != 0)
		{
			if(node_lidar.lidar_status.lidar_abnormal_state & 0x01)
			{
				pubdata.data="node_lidar is trapped\n";
				error_pub->publish(pubdata);
				printf("异常状态1---trapped\n");
			}
			if(node_lidar.lidar_status.lidar_abnormal_state & 0x02)
			{
				pubdata.data="node_lidar frequence abnormal\n";
				error_pub->publish(pubdata);
				printf("异常状态2---frequence abnormal\n");
			}
			if(node_lidar.lidar_status.lidar_abnormal_state & 0x04)
			{
				pubdata.data="node_lidar is blocked\n";
				error_pub->publish(pubdata);
				printf("异常状态3---blocked\n");
			}
			node_lidar.serial_port->write_data(end_lidar,4);
			node_lidar.lidar_status.lidar_ready = false;

			sleep(1);
		}
		LaserScan scan;
		if(data_handling(scan))
		{
			if(scan.points.size() == 0) {
				continue;
			}
			auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

			// Lidar mounting correction:
			// - Lidar 0° faces robot rear (180° offset from URDF X-axis)
			// - Lidar uses left-hand rule (CW), ROS uses right-hand (CCW) -> negate angles
			// - Valid FOV in lidar frame: 70°-290° (rest is occluded by chassis)
			//
			// Transform: ros_angle = -(lidar_angle - 180°) = 180° - lidar_angle
			// After transform, valid range becomes:
			//   lidar 70°  -> ros 110°   (= pi*110/180)
			//   lidar 290° -> ros -110°  (= -pi*110/180)
			// So ROS angles span from -110° to +110° (front-facing)

			const float DEG2RAD = M_PI / 180.0f;
			const float lidar_fov_min_deg = 70.0f;
			const float lidar_fov_max_deg = 290.0f;

			// ROS scan output: from -110° to +110°
			const float ros_angle_min = -110.0f * DEG2RAD;
			const float ros_angle_max = 110.0f * DEG2RAD;

			size_t num_raw = scan.points.size();
			// Live per-frame step; used only to reconstruct each raw point's
			// angle below (i * angle_inc_deg).
			float angle_inc_raw = scan.config.angle_increment;
			float angle_inc_deg = angle_inc_raw * 180.0f / M_PI;

			// Fixed output grid. The lidar's samples-per-revolution jitters by a
			// couple of points (motor RPM/load/temperature), so deriving the output
			// count from the live count made /scan alternate 244/245/246. slam_toolbox
			// locks to the first frame's size and drops every mismatched frame, which
			// froze the map during translation. Anchor the grid to a nominal ~401
			// pts/rev so every frame has a constant size; a short sweep simply leaves
			// trailing bins as inf (treated by slam as no-return).
			const float nominal_pts_per_rev = 401.0f;
			const float nominal_inc_raw = 2.0 * M_PI / nominal_pts_per_rev;
			int out_count = (int)((ros_angle_max - ros_angle_min) / nominal_inc_raw) + 1;
			float ros_angle_increment = (ros_angle_max - ros_angle_min) / (out_count - 1);

			scan_msg->ranges.resize(out_count, std::numeric_limits<float>::infinity());
			scan_msg->intensities.resize(out_count, 0.0f);

			// Stamp at scan START, not publish time: back off one full sweep
			// so slam_toolbox matches the scan against the correct odom pose.
			scan_msg->header.stamp =
				node->get_clock()->now()
				- rclcpp::Duration::from_seconds(scan.config.scan_time);
			scan_msg->header.frame_id = node_lidar.lidar_general_info.frame_id;

			scan_msg->angle_min = ros_angle_min;
			scan_msg->angle_max = ros_angle_max;
			scan_msg->angle_increment = ros_angle_increment;
			scan_msg->scan_time = scan.config.scan_time;
			scan_msg->time_increment = scan.config.time_increment;
			scan_msg->range_min = scan.config.min_range;
			scan_msg->range_max = scan.config.max_range;

			// Map each raw lidar point to the corrected ROS scan
			for (size_t i = 0; i < num_raw; i++) {
				// Raw lidar angle in degrees (0-360, left-hand/CW)
				float lidar_angle_deg = (float)i * angle_inc_deg;

				// Skip points outside valid FOV
				if (lidar_angle_deg < lidar_fov_min_deg || lidar_angle_deg > lidar_fov_max_deg)
					continue;

				// 雷达0°朝车尾反装(+SDK reversion=true 已使其呈CCW方向)：
				// v(α) = (-cos α, -sin α), 匹配ROS(cos β, sin β) -> β = α - 180°
				float ros_angle = (lidar_angle_deg - 180.0f) * DEG2RAD;

				// Find index in output array
				int idx = (int)((ros_angle - ros_angle_min) / ros_angle_increment + 0.5f);
				if (idx >= 0 && idx < out_count) {
					scan_msg->ranges[idx] = scan.points[i].range;
					scan_msg->intensities[idx] = scan.points[i].intensity;
				}
			}

			laser_pub->publish(*scan_msg);
			// printf("published scan: %d points (from %zu raw), subs=%zu\n",
			// 	out_count, num_raw, laser_pub->get_subscription_count());
			// fflush(stdout);
      	}
	}
	node_lidar.serial_port->write_data(end_lidar,4);
	return 0;
}

