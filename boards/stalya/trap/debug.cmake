set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
add_compile_options(-Wno-error=frame-larger-than=)

px4_add_board(
	PLATFORM nuttx
	VENDOR stalya
	MODEL trap
	LABEL debug
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m7
	ROMFSROOT trapfmu_test
	IO stalya_io-v2_default
	SERIAL_PORTS
		TEL1:/dev/ttyS0
		GPS2:/dev/ttyS1
		#CON:/dev/ttyS2
		TEL2:/dev/ttyS3
		#PX4IO:/dev/ttyS4
		TEL3:/dev/ttyS5
		GPS1:/dev/ttyS6
	DRIVERS
		adc
		gps
		imu/bmi088
		lights/blinkm
		lights/rgbled
		pwm_out_sim
		pwm_out
		px4io
		roboclaw
		rpm
		telemetry
		tone_alarm
		usbhub/usb251x
	MODULES
		airspeed_selector
		attitude_estimator_q
		battery_status
		commander
		dataman
		ekf2
		events
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
		vmount
	SYSTEMCMDS
		bl_update
		gpio
		hardfault_log
		led_control
		mixer
		mtd
		nshterm
		param
		pwm
		reboot
		top
		tune_control
		ver
		work_queue
	EXAMPLES
	)
