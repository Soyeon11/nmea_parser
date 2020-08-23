#include <ros/ros.h>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
// #include <tf/tf.h>
#include <iostream>
#include <string>
#include <cstring>
#include <cmath>

using namespace std;
// using namespace tf;


class NMEAParser {
private:
	ros::NodeHandle nh_;
	ros::Publisher fix_pub_;
	ros::Publisher course_pub_;
	ros::Subscriber nmea_sub_;
	
	sensor_msgs::NavSatFix fix_msg_;
	sensor_msgs::NavSatStatus nav_status_;

	ackermann_msgs::AckermannDriveStamped course_msg_;

	string nmea_sentence_;
	bool isRMC_;
	bool isVTG_;
	bool ischecksum_;
	bool isstatus_;
	double latitude_;
	bool isnorth_;
	double longitude_;
	bool iseast_;
	bool iscourse_over_ground_;
	bool isfirst_course_over_ground_;

	double prev_course_over_ground_;
	double course_over_ground_;

	// double initial_heading_;
	double true_heading_;
	bool istrue_heading_;
	bool istrue_;

public:	
	NMEAParser() {
		initSetup();
	}

	~NMEAParser() {
	}

	void initSetup() {
		isRMC_ = false;
		isVTG_ = false;
		ischecksum_ = false;
		isstatus_ = false;
		isnorth_ = true;
		iseast_ = true;
		iscourse_over_ground_ = false;
		isfirst_course_over_ground_ = true;

		istrue_heading_ = false;
		istrue_ = false;
		
		fix_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("fix", 10);
		latitude_ = 0.0;
		longitude_ = 0.0;

		prev_course_over_ground_ = 0.0;
		course_over_ground_ = 0.0;
		true_heading_ = 0.0;
		course_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("course", 10);

		nmea_sub_ = nh_.subscribe("nmea_sentence", 1, &NMEAParser::nmeaCallback, this);
		
	}

	void nmeaCallback(const nmea_msgs::Sentence::ConstPtr &nmea_msg) {
		nmea_sentence_ = nmea_msg->sentence;
		nmeaProcess(nmea_sentence_);
		if(isRMC_ && ischecksum_) {
			fix_msg_.header = nmea_msg->header;

			if(isstatus_) fix_msg_.status.status = nav_status_.STATUS_FIX;
			else fix_msg_.status.status = nav_status_.STATUS_NO_FIX;

			fix_msg_.status.service = nav_status_.SERVICE_GPS;
			
			if(isnorth_) fix_msg_.latitude = latitude_;
			else fix_msg_.latitude = -latitude_;

			if(iseast_) fix_msg_.longitude = longitude_;
			else fix_msg_.longitude = -longitude_;
		
			if(iscourse_over_ground_ || !isfirst_course_over_ground_) {

				course_msg_.header.frame_id = "gps";
				course_msg_.drive.steering_angle = course_over_ground_;
				course_pub_.publish(course_msg_);
				ROS_INFO("RMC COURSE OVER GROUND: %f", course_over_ground_);
			}

			fix_msg_.position_covariance_type = fix_msg_.COVARIANCE_TYPE_UNKNOWN;

			fix_pub_.publish(fix_msg_);
				
			isRMC_ = false;
			ischecksum_ = false;
			isstatus_ = false;
			isnorth_ = false;
			iseast_ = false;
			iscourse_over_ground_ = false;
		}
		else if(isVTG_ && ischecksum_) {
			if(istrue_heading_ && istrue_) ROS_INFO("VTG HEADING: %f", true_heading_); 
			isVTG_ = false;
			istrue_heading_ = false;
			istrue_ = false;
		}
	}
	
	int calcNMEAChecksum(char *buf, int len) {
		char character; 
		int checksum = 0; 
		int i;
		for (i=0; i < len; ++i) { 
			character = buf[i];
			switch(character) {
				case '$':
					break;
				case '*':
					i = len; 
					continue; 
				default:
					if (checksum == 0) {
						checksum = character;
					} 
					else {
						checksum = checksum ^ character;
					} 
					break;
			} 
		}
		return checksum;
	}

	

	void nmeaProcess(string buf) {
		int buf_size = buf.size();
		char buf_c[buf_size];
		strcpy(buf_c, buf.c_str());

		if(buf.find("RMC") != string::npos) {
			isRMC_ = true;
			string checksum = buf.substr(buf_size-2);
			int checksum_buf = stoi(checksum, 0, 16);
			int checksum_result = calcNMEAChecksum(buf_c, buf_size);
			
			if(checksum_buf == checksum_result) {
				// cout<<"VALID CHECKSUM."<<endl;
				ischecksum_ = true;
			}
			else {
				cout<<"INVALID CHECKSUM."<<endl;
				ischecksum_ = false;
				return;
			}
			
			string status;
			int pos = -1;
			int temp_pos;

			for(int i=0;i<8;i++) {
				pos = buf.find(",", pos+1);
				if(i==1) {
					status = buf.substr(pos+1, 1);
					if(status == "V") {
						// cout<<"INVALID DATA."<<endl;
						isstatus_ = false;
						// return;
					}
					else {
						isstatus_ = true;
						// cout<<"VALIDA DATA."<<endl;
					}
				}
				else if(i==2) {
					temp_pos = buf.find(",", pos+1);
					string lat = buf.substr(pos+1, temp_pos-pos-1);
					int dot_pos = lat.find(".");
					double deg = stof(lat.substr(0, dot_pos-2));
					double min = stof(lat.substr(dot_pos-2));
					latitude_ = deg + min / 60.0;
				}
				else if(i==3) {
					temp_pos = buf.find(",", pos+1);
					if(buf.substr(pos+1, temp_pos-pos-1)=="N") isnorth_ = true;
					else isnorth_ = false;
				}
				else if(i==4) {
					temp_pos = buf.find(",", pos+1);
					string lon = buf.substr(pos+1, temp_pos-pos-1);
					int dot_pos = lon.find(".");
					double deg = stof(lon.substr(0, dot_pos-2));
					double min = stof(lon.substr(dot_pos-2));
					longitude_ = deg + min / 60.0;
				}
				else if(i==5) {
					temp_pos = buf.find(",", pos+1);
					if(buf.substr(pos+1, temp_pos-pos-1)=="E") iseast_ = true;
					else iseast_ = false;		
				}
				else if(i==7) {
					temp_pos = buf.find(",", pos+1);
					if(buf.substr(pos+1, temp_pos-pos-1) != "") {
						isfirst_course_over_ground_ = false;
						iscourse_over_ground_ = true;
						course_over_ground_ = stof(buf.substr(pos+1, temp_pos-pos-1));
						prev_course_over_ground_ = course_over_ground_;
					}
					else {
						iscourse_over_ground_ = false;
						if(!isfirst_course_over_ground_) {
							course_over_ground_ = prev_course_over_ground_;
						}
					}
				}
			}
		}
		else if(buf.find("VTG") != string::npos) {
			isVTG_ = true;
			string checksum = buf.substr(buf_size-2);
			int checksum_buf = stoi(checksum, 0, 16);
			int checksum_result = calcNMEAChecksum(buf_c, buf_size);

			if(checksum_buf == checksum_result) {
				// cout<<"VALID CHECKSUM."<<endl;
				ischecksum_ = true;
			}
			else {
				cout<<"INVALID CHECKSUM."<<endl;
				ischecksum_ = false;
				return;
			}

			string status;
		       	int pos = -1;
			int temp_pos;

			for(int i=0;i<2;i++) {
				pos = buf.find(",", pos+1);
				if(i==0) {
					temp_pos = buf.find(",", pos+1);
					if(buf.substr(pos+1, temp_pos-pos-1) != "") {
						true_heading_ = stof(buf.substr(pos+1, temp_pos-pos-1));
						istrue_heading_ = true;
					}
					istrue_heading_ = false;
				}
				if(i==1) {
					temp_pos = buf.find(",", pos+1);
					if(buf.substr(pos+1, temp_pos-pos-1) == "T") istrue_ = true;
					else istrue_ = false;
				}
			}	
		}
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "nmea_parser");
	NMEAParser ps;
	ros::Rate loop_rate(10);

	while(ros::ok()) {
		ros::spin();
		loop_rate.sleep();
	}

	return 0;
}
