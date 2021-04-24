set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
add_compile_options(-Wno-error=frame-larger-than=)

px4_add_board(
	PLATFORM nuttx
	VENDOR stalya
	MODEL trap
	LABEL debug
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m7
	ROMFSROOT trapfmu
	CONSTRAINED_FLASH
	IO stalya_io-v2_default
	#UAVCAN_INTERFACES 2
	#UAVCAN_TIMER_OVERRIDE 2
	SERIAL_PORTS
		TEL1:/dev/ttyS0
		GPS1:/dev/ttyS1
		#CON:/dev/ttyS2
		TEL2:/dev/ttyS3
		#PX4IO:/dev/ttyS4
		TEL3:/dev/ttyS5
		GPS2:/dev/ttyS6
	DRIVERS
		adc/board_adc
		#barometer
		#batt_smbus
		#camera_capture
		#camera_trigger
		#differential_pressure
		#distance_sensor
		dshot
		gps
		#heater
		imu/bmi088
		#irlock
		lights/blinkm
		lights/rgbled
		#lights/rgbled_pwm
		#magnetometer
		mkblctrl
		#optical_flow
		pwm_out_sim
		pwm_out
		#rc_input
		px4io
		roboclaw
		#safety_button
		rpm
		#tap_esc
		telemetry
		tone_alarm
		#uavcan
		usbhub/usb251x
	MODULES
		airspeed_selector
		attitude_estimator_q
		battery_status
		#camera_feedback
		commander
		dataman
		ekf2
		#esc_battery
		events
		flight_mode_manager
		fw_att_control
		fw_pos_control_l1
		land_detector
		landing_target_estimator
		load_mon
		local_position_estimator
		logger
		mavlink
		mc_att_control
		mc_hover_thrust_estimator
		mc_pos_control
		mc_rate_control
		navigator
		rc_update
		rover_pos_control
		sensors
		sih
		#temperature_compensation
		vmount
		#vtol_att_control
	SYSTEMCMDS
		bl_update
		dmesg
		dumpfile
		esc_calib
		gpio
		hardfault_log
		#i2cdetect
		led_control
		mft
		mixer
		motor_ramp
		mtd
		nshterm
		param
		perf
		pwm
		reboot
		reflect
		system_time
		top
		topic_listener
		tune_control
		usb_connected
		ver
		work_queue
	EXAMPLES
	)
