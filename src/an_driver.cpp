/****************************************************************/
/*                                                              */
/*          Advanced Navigation Packet Protocol Library         */
/*        ROS Driver, Packet to Published Message Example       */
/*          Copyright 2020, Advanced Navigation Pty Ltd         */
/*                                                              */
/****************************************************************/
/*
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <time.h>

#include "NTRIP_Client/NTRIP/ntripclient.h"
#include "rs232/rs232.h"
#include "an_packet_protocol.h"
#include "spatial_packets.h"

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/FluidPressure.h>
#include <std_msgs/String.h>
#define RADIANS_TO_DEGREES (180.0/M_PI)
const double PI = 4*atan(1);
static const size_t MAX_NMEA_STRING_LENGTH = 256;
struct tm * timeinfoUnix;

int generateGPGGA(char* buf, system_state_packet_t& state){
	uint64_t time = state.unix_time_seconds * 1000000;//unix_time_microseconds();
	int subseconds = lroundf((time % 1000000L) / 1000.0f);
	if(subseconds == 1000){
		time +=1000000L;
    	subseconds = 0;
	}
	/*if(state.unix_time_seconds.accuracy() > TIME_ACCURACY_GNSS_COARSE){
		subseconds = lroundf(subseconds / 20.0f) * 20;
	}*/
	time_t unix_time = time / 1000000;
	timeinfoUnix = gmtime(&unix_time);
	
	const auto latitude = state.latitude * (180.0f / M_PI);
	const auto longitude = state.longitude * (180.0f / M_PI);
	char latitude_character = 'N';
	char longitude_character = 'E';
	if(latitude < 0){
		latitude_character = 'S';
	}
	if(longitude < 0){
		longitude_character = 'W';
	}

	// These values are hard codedas it is necessary for the Base Station.
	int nmea_fix_type = 1;
	int satellites = 10;
	float dop = 2;

	int n = snprintf(buf, 
		MAX_NMEA_STRING_LENGTH,
		"GPGGA,%02d%02d%02d.%03d,%02d%010.7f,%c,%03d%010.7f,%c,%d,%02d,%.1f,%.3f,M,,M,,*",
		timeinfoUnix->tm_hour,
		timeinfoUnix->tm_min,
		timeinfoUnix->tm_sec,
		subseconds,
		abs((int) latitude),
		fabs(latitude - (int) latitude) * 60.0,
		latitude_character,
		abs((int) longitude),
		fabs(longitude - (int) longitude) * 60.0,
		longitude_character,
		nmea_fix_type,
		satellites,
		dop,
		state.height);

	uint8_t checksum = 0;
	for(uint16_t i = 1; i < n - 1; ++i){
		checksum ^= (const uint8_t) buf[i];
	}
	n += snprintf(&buf[n], MAX_NMEA_STRING_LENGTH - n, "%02X\r\n", checksum);
	return n;
	
}

int main(int argc, char *argv[]) {
	
	// Set up ROS node //
	ros::init(argc, argv, "an_device_node");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	//For Debugging
	printf("argc: %d\n", argc);
	for(int i = 0; i<argc; i++){
		printf("argv[%d]: %s\n", i, argv[i]);
	}
	
	printf("\nYour Advanced Navigation ROS driver is currently running\nPress Ctrl-C to interrupt\n");

	// Set up for Log File
	static const uint8_t request_all_configuration[] = { 0xE2, 0x01, 0x10, 0x9A, 0x73, 0xB6, 0xB4, 0xB5, 0xB8, 0xB9, 0xBA, 0xBC, 0xBD, 0xC0, 0xC2, 0xC3, 0xC4, 0x03, 0xC6, 0x45, 0xC7 };
	
	FILE *log_file;
	char filename[32];
	time_t rawtime;
	struct tm * timeinfo;
	int write_counter = 0;
	

	// Set up the COM port
	int baud_rate;
	std::string com_port;
	std::string imu_frame_id;
	std::string nav_sat_frame_id;
	std::string rawsensors_magnetometer_frame_id;
	std::string barometric_pressure_frame_id;
	std::string temperature_frame_id;
	std::string gnss_fix_type_id;
	std::string topic_prefix;
	std::stringstream gnssFixType;
	tf::Quaternion orientation;
	std_msgs::String gnss_fix_type_msgs;

	// NTRIP Varialbles	
	int error = 0;
	int bytes_received;
	int numbytes = 0;
	int remain = numbytes;
	int pos = 0;
	int state; // 0 = NTRIP Info provided, 1 = No Arguement, 3 = Baudrate and Port for serial 
	struct Args args;
	char buf[MAXDATASIZE];

	// Intialising for the log files
	rawtime = time(NULL);
	timeinfo = localtime(&rawtime);
	sprintf(filename, "Log_%02d-%02d-%02d_%02d-%02d-%02d.anpp", timeinfo->tm_year-100, timeinfo->tm_mon+1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
	log_file = fopen(filename, "wb");

	// Configuring based on the state, what sort of driver to run
	if (argc == 1){
		printf("argc: %d\n", argc);
		pnh.param("port", com_port, std::string("/dev/ttyUSB0"));
		pnh.param("baud_rate", baud_rate, 115200);
		state = 1;
	}
	else if (argc == 3) {
		com_port = std::string(argv[2]);
		baud_rate = atoi(argv[1]);
		state = 3;
	}
	else{
		getargs(argc, argv, &args);
		state = 0;
	}

	pnh.param("imu_frame_id", imu_frame_id, std::string("imu"));
	pnh.param("nav_sat_frame_id", nav_sat_frame_id, std::string("gps"));
	pnh.param("rawsensors_magnetometer_frame_id", rawsensors_magnetometer_frame_id, std::string("rawsensors_magnetometer"));
	pnh.param("barometric_pressure_frame_id", barometric_pressure_frame_id, std::string("barometric_pressure"));
	pnh.param("temperature_frame_id", temperature_frame_id, std::string("temperature"));
	pnh.param("topic_prefix", topic_prefix, std::string("an_device"));
	pnh.param("GNSS_Fix_Type", gnss_fix_type_id, std::string("gnss_fix_type"));

	// Initialise Publishers and Topics //
	ros::Publisher imu_pub=nh.advertise<sensor_msgs::Imu>(topic_prefix + "/Imu",10);
	ros::Publisher nav_sat_fix_pub=nh.advertise<sensor_msgs::NavSatFix>(topic_prefix + "/NavSatFix",10);
	ros::Publisher magnetic_field_pub=nh.advertise<sensor_msgs::MagneticField>(topic_prefix + "/MagneticField", 10);
	ros::Publisher barometric_pressure_pub=nh.advertise<sensor_msgs::FluidPressure>(topic_prefix + "/BarometricPressure", 10);
	ros::Publisher temperature_pub=nh.advertise<sensor_msgs::Temperature>(topic_prefix + "/Temperature", 10);	
	ros::Publisher twist_pub=nh.advertise<geometry_msgs::Twist>(topic_prefix + "/Twist",10);
	ros::Publisher pose_pub=nh.advertise<geometry_msgs::Pose>(topic_prefix + "/Pose", 10);
	ros::Publisher system_status_pub=nh.advertise<diagnostic_msgs::DiagnosticStatus>(topic_prefix + "/SystemStatus",10);
	ros::Publisher filter_status_pub=nh.advertise<diagnostic_msgs::DiagnosticStatus>(topic_prefix + "/FilterStatus",10);
	ros::Publisher gnss_fix_type_pub=nh.advertise<std_msgs::String>(topic_prefix + "/GNSSFixType", 10);

	#pragma region // Initialise messages

	// IMU sensor_msgs/Imu
	sensor_msgs::Imu imu_msg;
	imu_msg.header.stamp.sec = 0;
	imu_msg.header.stamp.nsec = 0;
	imu_msg.header.frame_id = '0';
	imu_msg.orientation.x =0.0;
	imu_msg.orientation.y = 0.0;
	imu_msg.orientation.z =0.0;
	imu_msg.orientation.w =0.0;
	imu_msg.orientation_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	imu_msg.angular_velocity.x = 0.0;
	imu_msg.angular_velocity.y = 0.0;
	imu_msg.angular_velocity.z = 0.0;
	imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // fixed
	imu_msg.linear_acceleration.x = 0.0;
	imu_msg.linear_acceleration.y = 0.0;
	imu_msg.linear_acceleration.z = 0.0;
	imu_msg.linear_acceleration_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // fixed

	// NavSatFix sensor_msgs/NavSatFix 
	sensor_msgs::NavSatFix nav_sat_fix_msg;
	nav_sat_fix_msg.header.stamp.sec = 0;
	nav_sat_fix_msg.header.stamp.nsec = 0;
	nav_sat_fix_msg.header.frame_id = '0';
	nav_sat_fix_msg.status.status = 0;
	nav_sat_fix_msg.status.service = 1; // fixed to GPS
	nav_sat_fix_msg.latitude = 0.0;
	nav_sat_fix_msg.longitude = 0.0;
	nav_sat_fix_msg.altitude = 0.0;
	nav_sat_fix_msg.position_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	nav_sat_fix_msg.position_covariance_type = 2; // fixed to variance on the diagonal

	// MagneticField geometry_msg/magnetic_field
	sensor_msgs::MagneticField magnetic_field_msg;
	magnetic_field_msg.magnetic_field.x = 0;
	magnetic_field_msg.magnetic_field.y = 0;
	magnetic_field_msg.magnetic_field.z = 0;

	// Barometric Pressure sensor_msgs/fluidPressure
	sensor_msgs::FluidPressure barometric_pressure_msg;
	barometric_pressure_msg.fluid_pressure = 0;

	// Temperature sensor_msgs/Temperature
	sensor_msgs::Temperature temperature_msg;
	temperature_msg.temperature = 0;

	// Twist sensor_msgs/twist
	geometry_msgs::Twist twist_msg;
	twist_msg.linear.x = 0.0;
	twist_msg.linear.y = 0.0;
	twist_msg.linear.z = 0.0;
	twist_msg.angular.x = 0.0;
	twist_msg.angular.y = 0.0;
	twist_msg.angular.z = 0.0;

	// Position in ECEF Postion (Packet 33) and Orientation in Quartenion Format (Same as IMU)
	geometry_msgs::Pose pose_msg;
	pose_msg.position.x = 0;
	pose_msg.position.y = 0;
	pose_msg.position.z = 0;
	pose_msg.orientation.x = 0.0;
	pose_msg.orientation.y = 0.0;
	pose_msg.orientation.z = 0.0;
	pose_msg.orientation.w = 0.0;

	// DiagnosticsStatus messages for System Status
	diagnostic_msgs::DiagnosticStatus system_status_msg;
	system_status_msg.level = 0; // default OK state
	system_status_msg.name = "System Status";
	system_status_msg.message = "";

	// DiagnosticsStatus messages for Filter Status
	diagnostic_msgs::DiagnosticStatus filter_status_msg;
	filter_status_msg.level = 0; // default OK state
	filter_status_msg.name = "Filter Status";
	filter_status_msg.message = "";

	// Initialise packets
	an_decoder_t an_decoder;
	an_packet_t *an_packet;
	system_state_packet_t system_state_packet;
	quaternion_orientation_standard_deviation_packet_t quaternion_orientation_standard_deviation_packet;
	raw_sensors_packet_t raw_sensors_packet;
	ecef_position_packet_t ecef_position_packet;
	#pragma endregion

	// Opening the COM Port
	if(state == 0){
		if(OpenComport(args.serdevice, args.baud)){
			printf("Could not open serial port\n");
			exit(EXIT_FAILURE);			
		}
		error = ntrip_initialise(&args, buf);
		if(error){
			printf("ERROR\n");
		}
		else{
			//printf("NOT ERROR\n");
			error += 0;
		}
	}
	if(state == 1 || state == 3){
		if(OpenComport(const_cast<char*>(com_port.c_str()), baud_rate))
		{
			printf("Could not open serial port: %s \n",com_port.c_str());
			exit(EXIT_FAILURE);
		}
	}
	// Request Config packets and also start decoding anpp packets
	SendBuf((unsigned char*)request_all_configuration, sizeof(request_all_configuration));
	an_decoder_initialise(&an_decoder);
		

	// Loop continuously, polling for packets
	while(ros::ok() && !error)
	{
		ros::spinOnce();
		bytes_received = PollComport(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder));
		if(bytes_received > 0)
		{
			fwrite(an_decoder_pointer(&an_decoder), sizeof(uint8_t), bytes_received, log_file);
			// Increment the decode buffer length by the number of bytes received 
			an_decoder_increment(&an_decoder, bytes_received);

			// Decode all the packets in the buffer 
			while((an_packet = an_packet_decode(&an_decoder)) != NULL)
			{				
				#pragma region	// Packet Decoding
				// System State Packet Decoding
				if(an_packet->id == packet_id_system_state)
				{
					if(decode_system_state_packet(&system_state_packet, an_packet) == 0)
					{
						// GNSS FIX TYPE
						switch(system_state_packet.filter_status.b.gnss_fix_type)
						{
							case 0: 
								gnssFixType.str("No GNSS fix");
								break;
							case 1:
								gnssFixType.str("2D Fix");
								break;
							case 2:
								gnssFixType.str("3D Fix");
								break;
							case 3:
								gnssFixType.str("SBAS Fix");
								break;
							case 4:
								gnssFixType.str("Differential Fix");
								break;
							case 5:
								gnssFixType.str("Omnistar/Starfire Fix");
								break;
							case 6:
								gnssFixType.str("RTK Float");
								break;
							case 7:
								gnssFixType.str("RTK Fixed");
								break;
							default:
								gnssFixType.str("NOT CONNECTED");
						}						
						gnss_fix_type_msgs.data = gnssFixType.str();

						// NAVSATFIX
						nav_sat_fix_msg.header.stamp.sec=system_state_packet.unix_time_seconds;
						nav_sat_fix_msg.header.stamp.nsec=system_state_packet.microseconds*1000;
						nav_sat_fix_msg.header.frame_id=nav_sat_frame_id;
						if((system_state_packet.filter_status.b.gnss_fix_type == 1) || 
							(system_state_packet.filter_status.b.gnss_fix_type == 2))
						{
							nav_sat_fix_msg.status.status=0;
						}
						else if((system_state_packet.filter_status.b.gnss_fix_type == 3) ||
							 (system_state_packet.filter_status.b.gnss_fix_type == 5))
						{
							nav_sat_fix_msg.status.status=1;
						}
						else if((system_state_packet.filter_status.b.gnss_fix_type == 4) ||
							 (system_state_packet.filter_status.b.gnss_fix_type == 6) ||
							 (system_state_packet.filter_status.b.gnss_fix_type == 7))
						{
							nav_sat_fix_msg.status.status=2;
						}
						else
						{
							nav_sat_fix_msg.status.status=-1;
						}
						nav_sat_fix_msg.latitude=system_state_packet.latitude * RADIANS_TO_DEGREES;
						nav_sat_fix_msg.longitude=system_state_packet.longitude * RADIANS_TO_DEGREES;
						nav_sat_fix_msg.altitude=system_state_packet.height;
						nav_sat_fix_msg.position_covariance={pow(system_state_packet.standard_deviation[1],2), 0.0, 0.0,
							0.0, pow(system_state_packet.standard_deviation[0],2), 0.0,
							0.0, 0.0, pow(system_state_packet.standard_deviation[2],2)};						


						// TWIST
						twist_msg.linear.x = system_state_packet.velocity[0];
						twist_msg.linear.y = system_state_packet.velocity[1];
						twist_msg.linear.z = system_state_packet.velocity[2];
						twist_msg.angular.x = system_state_packet.angular_velocity[0];
						twist_msg.angular.y = system_state_packet.angular_velocity[1];
						twist_msg.angular.z = system_state_packet.angular_velocity[2];


						// IMU
						imu_msg.header.stamp.sec = system_state_packet.unix_time_seconds;
						imu_msg.header.stamp.nsec = system_state_packet.microseconds * 1000;
						imu_msg.header.frame_id = imu_frame_id;
						// Using the RPY orientation as done by cosama
						orientation.setRPY(
							system_state_packet.orientation[0],
							system_state_packet.orientation[1],
							PI/2.0f - system_state_packet.orientation[2] //REP 103
						);
						imu_msg.orientation.x = orientation[0];
						imu_msg.orientation.y = orientation[1];
						imu_msg.orientation.z = orientation[2];
						imu_msg.orientation.w = orientation[3];

						// POSE Orientation
						pose_msg.orientation.x = orientation[0];
						pose_msg.orientation.y = orientation[1];
						pose_msg.orientation.z = orientation[2];
						pose_msg.orientation.w = orientation[3];

						imu_msg.angular_velocity.x = system_state_packet.angular_velocity[0]; // These the same as the TWIST msg values
						imu_msg.angular_velocity.y = system_state_packet.angular_velocity[1];
						imu_msg.angular_velocity.z = system_state_packet.angular_velocity[2];

						// SYSTEM STATUS
						system_status_msg.message = "";
						system_status_msg.level = 0; // default OK state
						if(system_state_packet.system_status.b.system_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message += "0. System Failure! ";
						}
						if(system_state_packet.system_status.b.accelerometer_sensor_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message  += "1. Accelerometer Sensor Failure! ";
						}
						if(system_state_packet.system_status.b.gyroscope_sensor_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message  +=  "2. Gyroscope Sensor Failure! ";
						}
						if(system_state_packet.system_status.b.magnetometer_sensor_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message  += "3. Magnetometer Sensor Failure! ";
						}
						if(system_state_packet.system_status.b.pressure_sensor_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message  += "4. Pressure Sensor Failure! ";
						}
						if(system_state_packet.system_status.b.gnss_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message  += "5. GNSS Failure! ";
						}
						if(system_state_packet.system_status.b.accelerometer_over_range) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message  += "6. Accelerometer Over Range! ";
						}
						if(system_state_packet.system_status.b.gyroscope_over_range) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message  += "7. Gyroscope Over Range! ";
						}
						if(system_state_packet.system_status.b.magnetometer_over_range) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message  += "8. Magnetometer Over Range! ";
						}
						if(system_state_packet.system_status.b.pressure_over_range) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message  += "9. Pressure Over Range! ";
						}
						if(system_state_packet.system_status.b.minimum_temperature_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message  += "10. Minimum Temperature Alarm! ";
						}
						if(system_state_packet.system_status.b.maximum_temperature_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message  += "11. Maximum Temperature Alarm! ";
						}
						if(system_state_packet.system_status.b.low_voltage_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message  += "12. Low Voltage Alarm! ";
						}
						if(system_state_packet.system_status.b.high_voltage_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message  += "13. High Voltage Alarm! ";
						}
						if(system_state_packet.system_status.b.gnss_antenna_disconnected) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message  += "14. GNSS Antenna Disconnected! ";
						}
						if(system_state_packet.system_status.b.serial_port_overflow_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message  += "15. Data Output Overflow Alarm! ";
						}

						// FILTER STATUS
						filter_status_msg.message = "";
						filter_status_msg.level = 0; // default OK state
						if(system_state_packet.filter_status.b.orientation_filter_initialised) {
							filter_status_msg.message  += "0. Orientation Filter Initialised. ";
						}
						else{
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message  += "0. Orientation Filter NOT Initialised. ";
						}
						if(system_state_packet.filter_status.b.ins_filter_initialised) {
							filter_status_msg.message  += "1. Navigation Filter Initialised. ";
						}
						else{
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message  += "1. Navigation Filter NOT Initialised. ";
						}
						if(system_state_packet.filter_status.b.heading_initialised) {
							filter_status_msg.message  += "2. Heading Initialised. ";
						}
						else{
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message  += "2. Heading NOT Initialised. ";
						}
						if(system_state_packet.filter_status.b.utc_time_initialised) {
							filter_status_msg.message  += "3. UTC Time Initialised. ";
						}
						else{
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message  += "3. UTC Time NOT Initialised. ";
						}
						if(system_state_packet.filter_status.b.event1_flag) {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message  += "7. Event 1 Occured. ";
						}
						else{
							filter_status_msg.message  += "7. Event 1 NOT Occured. ";
						}
						if(system_state_packet.filter_status.b.event2_flag) {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message  += "8. Event 2 Occured. ";
						}
						else{
							filter_status_msg.message  += "8. Event 2 NOT Occured. ";
						}
						if(system_state_packet.filter_status.b.internal_gnss_enabled) {
							filter_status_msg.message  += "9. Internal GNSS Enabled. ";
						}
						else{
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message  += "9. Internal GNSS NOT Enabled. ";
						}
						if(system_state_packet.filter_status.b.magnetic_heading_enabled) {
							filter_status_msg.message  += "10. Magnetic Heading Active. ";
						}
						else{
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message  += "10. Magnetic Heading NOT Active. ";
						}
						if(system_state_packet.filter_status.b.velocity_heading_enabled) {
							filter_status_msg.message  += "11. Velocity Heading Enabled. ";
						}
						else{
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message  += "11. Velocity Heading NOT Enabled. ";
						}
						if(system_state_packet.filter_status.b.atmospheric_altitude_enabled) {
							filter_status_msg.message  += "12. Atmospheric Altitude Enabled. ";
						}
						else{
							filter_status_msg.message  += "12. Atmospheric Altitude NOT Enabled. ";
							filter_status_msg.level = 1; // WARN state
						}
						if(system_state_packet.filter_status.b.external_position_active) {
							filter_status_msg.message  += "13. External Position Active. ";
						}
						else{
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message  += "13. External Position NOT Active. ";
						}
						if(system_state_packet.filter_status.b.external_velocity_active) {
							filter_status_msg.message  += "14. External Velocity Active. ";
						}
						else{
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message  += "14. External Velocity NOT Active. ";
						}
						if(system_state_packet.filter_status.b.external_heading_active) {
							filter_status_msg.message  += "15. External Heading Active. ";
						}
						else{
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message  += "15. External Heading NOT Active. ";
						}
					}
				}
				#pragma endregion

				// ECEF Position (in meters) Packet for Pose Message
				if(an_packet->id == packet_id_ecef_position)
				{		
					if(decode_ecef_position_packet(&ecef_position_packet, an_packet) == 0)
					{
						pose_msg.position.x = ecef_position_packet.position[0];
						pose_msg.position.y = ecef_position_packet.position[1];
						pose_msg.position.z = ecef_position_packet.position[2];
					}
				}

				// QUATERNION ORIENTATION STANDARD DEVIATION PACKET 
				if(an_packet->id == packet_id_quaternion_orientation_standard_deviation)
				{
					// copy all the binary data into the typedef struct for the packet 
					// this allows easy access to all the different values             
					if(decode_quaternion_orientation_standard_deviation_packet(&quaternion_orientation_standard_deviation_packet, an_packet) == 0)
					{
						// IMU
						imu_msg.orientation_covariance[0] = quaternion_orientation_standard_deviation_packet.standard_deviation[0];
						imu_msg.orientation_covariance[4] = quaternion_orientation_standard_deviation_packet.standard_deviation[1];
						imu_msg.orientation_covariance[8] = quaternion_orientation_standard_deviation_packet.standard_deviation[2];
					}
				}

				// Setting up the magnetic field to display
				if((an_packet->id == packet_id_raw_sensors) && (decode_raw_sensors_packet(&raw_sensors_packet, an_packet) == 0)){
					// Time Stamp from the System State Packet
					magnetic_field_msg.header.stamp.sec = system_state_packet.unix_time_seconds;
					barometric_pressure_msg.header.stamp.sec = system_state_packet.unix_time_seconds;
					temperature_msg.header.stamp.sec = system_state_packet.unix_time_seconds;
					magnetic_field_msg.header.stamp.nsec = system_state_packet.microseconds*1000;
					barometric_pressure_msg.header.stamp.nsec = system_state_packet.microseconds*1000;
					temperature_msg.header.stamp.nsec = system_state_packet.microseconds*1000;
				
					// RAW MAGNETICFIELD VALUE FROM IMU
					magnetic_field_msg.header.frame_id = rawsensors_magnetometer_frame_id;
					magnetic_field_msg.magnetic_field.x = raw_sensors_packet.magnetometers[0];
					magnetic_field_msg.magnetic_field.y = raw_sensors_packet.magnetometers[1];
					magnetic_field_msg.magnetic_field.z = raw_sensors_packet.magnetometers[2];
					imu_msg.linear_acceleration.x = raw_sensors_packet.accelerometers[0];
					imu_msg.linear_acceleration.y = raw_sensors_packet.accelerometers[1];
					imu_msg.linear_acceleration.z = raw_sensors_packet.accelerometers[2];

					// BAROMETRIC PRESSURE
					barometric_pressure_msg.header.frame_id = barometric_pressure_frame_id;
					barometric_pressure_msg.fluid_pressure = raw_sensors_packet.pressure;

					// TEMPERATURE
					temperature_msg.header.frame_id = rawsensors_magnetometer_frame_id;
					temperature_msg.temperature = raw_sensors_packet.pressure_temperature;
					
				}

				// Ensure that you free the an_packet when your done with it or you will leak memory                                  
				an_packet_free(&an_packet);

				// PUBLISH MESSAGES
				nav_sat_fix_pub.publish(nav_sat_fix_msg);
				twist_pub.publish(twist_msg);
				imu_pub.publish(imu_msg);
				system_status_pub.publish(system_status_msg);
				filter_status_pub.publish(filter_status_msg);
				magnetic_field_pub.publish(magnetic_field_msg);
				barometric_pressure_pub.publish(barometric_pressure_msg);
				temperature_pub.publish(temperature_msg);
				pose_pub.publish(pose_msg);
				gnss_fix_type_pub.publish(gnss_fix_type_msgs);
			}
			
			// Write the logs to the logger reset when counter is full
			if(write_counter++ >= 100){
				fflush(log_file);
				write_counter = 0;
			}
		}
		
		char nmea[MAX_NMEA_STRING_LENGTH];
		int n = generateGPGGA(nmea, system_state_packet);
		ROS_WARN_STREAM(nmea);		
		args.nmea = nmea;
		/*getargs(argc, argv, &args);
		error = ntrip_initialise(&args, buf);
		if(error){
			printf("ERROR\n");
		}
		else{
			//printf("NOT ERROR\n");
			error += 0;
		}*/
		
		
		if(state == 0){
			error = ntrip(&args, buf, &numbytes);
			remain = numbytes;
			
			// Send Buffer in 255 Byte chunks to the Spatial 
			// Loop till the entire rtcm corrections message is encoded. 
			while(remain)
			{	
				int toCpy = remain > AN_MAXIMUM_PACKET_SIZE ? AN_MAXIMUM_PACKET_SIZE : remain;
				an_packet = encode_rtcm_corrections_packet(toCpy, buf+pos);				
				an_packet_encode(an_packet);
				SendBuf(an_packet_pointer(an_packet), an_packet_size(an_packet));
				an_packet_free(&an_packet);
				pos += toCpy;			

				// Increment buffer
				remain -= toCpy;
			}			
			pos=0;
		}
	}
}



