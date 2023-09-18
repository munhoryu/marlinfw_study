// MarlinCore.cpp
#include "MarlinCore.h"
//#include "HAL/shared/Delay.h"
//#include "HAL/shared/esp_wifi.h"
//#include "HAL/shared/cpu_exception/exception_hook.h"

void setup(void) {
/*
#ifdef FASTIO_INIT
	FASTIO_INIT();
#endif

#ifdef BOARD_PREINIT
	BOARD_PREINIT(); // Low-level init (before serial init)
#endif

	tmc_standby_setup();  // TMC Low Power Standby pins must be set early or they're not usable

	// Check startup - does nothing if bootloader sets MCUSR to 0
	const byte mcu = hal.get_reset_source();
	hal.clear_reset_source();

#if ENABLED(MARLIN_DEV_MODE)
	auto log_current_ms = [&](PGM_P const msg) {
		SERIAL_ECHO_START();
		SERIAL_CHAR('['); SERIAL_ECHO(millis()); SERIAL_ECHOPGM("] ");
		SERIAL_ECHOLNPGM_P(msg);
	};
#define SETUP_LOG(M) log_current_ms(PSTR(M))
#else
#define SETUP_LOG(...) NOOP
#endif
#define SETUP_RUN(C) do{ SETUP_LOG(STRINGIFY(C)); C; }while(0)

	MYSERIAL1.begin(BAUDRATE);
	millis_t serial_connect_timeout = millis() + 1000UL;
	while (!MYSERIAL1.connected() && PENDING(millis(), serial_connect_timeout)) { //nada }

#if HAS_MULTI_SERIAL && !HAS_ETHERNET
#ifndef BAUDRATE_2
#define BAUDRATE_2 BAUDRATE
#endif
	MYSERIAL2.begin(BAUDRATE_2);
	serial_connect_timeout = millis() + 1000UL;
	while (!MYSERIAL2.connected() && PENDING(millis(), serial_connect_timeout)) { //nada }
#ifdef SERIAL_PORT_3
#ifndef BAUDRATE_3
#define BAUDRATE_3 BAUDRATE
#endif
	MYSERIAL3.begin(BAUDRATE_3);
	serial_connect_timeout = millis() + 1000UL;
	while (!MYSERIAL3.connected() && PENDING(millis(), serial_connect_timeout)) { //nada }
#endif
#endif
	SERIAL_ECHOLNPGM("start");

	// Set up these pins early to prevent suicide
#if HAS_KILL
	SETUP_LOG("KILL_PIN");
#if KILL_PIN_STATE
	SET_INPUT_PULLDOWN(KILL_PIN);
#else
	SET_INPUT_PULLUP(KILL_PIN);
#endif
#endif

#if ENABLED(FREEZE_FEATURE)
	SETUP_LOG("FREEZE_PIN");
#if FREEZE_STATE
	SET_INPUT_PULLDOWN(FREEZE_PIN);
#else
	SET_INPUT_PULLUP(FREEZE_PIN);
#endif
#endif

#if HAS_SUICIDE
	SETUP_LOG("SUICIDE_PIN");
	OUT_WRITE(SUICIDE_PIN, !SUICIDE_PIN_STATE);
#endif

#ifdef JTAGSWD_RESET
	SETUP_LOG("JTAGSWD_RESET");
	JTAGSWD_RESET();
#endif

	// Disable any hardware debug to free up pins for IO
#if ENABLED(DISABLE_DEBUG) && defined(JTAGSWD_DISABLE)
	delay(10);
	SETUP_LOG("JTAGSWD_DISABLE");
	JTAGSWD_DISABLE();
#elif ENABLED(DISABLE_JTAG) && defined(JTAG_DISABLE)
	delay(10);
	SETUP_LOG("JTAG_DISABLE");
	JTAG_DISABLE();
#endif

	TERN_(DYNAMIC_VECTORTABLE, hook_cpu_exceptions()); // If supported, install Marlin exception handlers at runtime

	SETUP_RUN(hal.init());

	// Init and disable SPI thermocouples; this is still needed
#if TEMP_SENSOR_IS_MAX_TC(0) || (TEMP_SENSOR_IS_MAX_TC(REDUNDANT) && REDUNDANT_TEMP_MATCH(SOURCE, E0))
	OUT_WRITE(TEMP_0_CS_PIN, HIGH);  // Disable
#endif
#if TEMP_SENSOR_IS_MAX_TC(1) || (TEMP_SENSOR_IS_MAX_TC(REDUNDANT) && REDUNDANT_TEMP_MATCH(SOURCE, E1))
	OUT_WRITE(TEMP_1_CS_PIN, HIGH);
#endif

#if ENABLED(DUET_SMART_EFFECTOR) && PIN_EXISTS(SMART_EFFECTOR_MOD)
	OUT_WRITE(SMART_EFFECTOR_MOD_PIN, LOW);   // Put Smart Effector into NORMAL mode
#endif

#if HAS_FILAMENT_SENSOR
	SETUP_RUN(runout.setup());
#endif

#if HAS_TMC220x
	SETUP_RUN(tmc_serial_begin());
#endif

#if HAS_TMC_SPI
#if DISABLED(TMC_USE_SW_SPI)
	SETUP_RUN(SPI.begin());
#endif
	SETUP_RUN(tmc_init_cs_pins());
#endif

#if ENABLED(PSU_CONTROL)
	SETUP_LOG("PSU_CONTROL");
	powerManager.init();
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
	SETUP_RUN(recovery.setup());
#endif

#if HAS_STEPPER_RESET
	SETUP_RUN(disableStepperDrivers());
#endif

	SETUP_RUN(hal.init_board());

#if ENABLED(WIFISUPPORT)
	SETUP_RUN(esp_wifi_init());
#endif

	// Report Reset Reason
	if (mcu & RST_POWER_ON)  SERIAL_ECHOLNPGM(STR_POWERUP);
	if (mcu & RST_EXTERNAL)  SERIAL_ECHOLNPGM(STR_EXTERNAL_RESET);
	if (mcu & RST_BROWN_OUT) SERIAL_ECHOLNPGM(STR_BROWNOUT_RESET);
	if (mcu & RST_WATCHDOG)  SERIAL_ECHOLNPGM(STR_WATCHDOG_RESET);
	if (mcu & RST_SOFTWARE)  SERIAL_ECHOLNPGM(STR_SOFTWARE_RESET);

	// Identify myself as Marlin x.x.x
	SERIAL_ECHOLNPGM("Marlin " SHORT_BUILD_VERSION);
#if defined(STRING_DISTRIBUTION_DATE) && defined(STRING_CONFIG_H_AUTHOR)
	SERIAL_ECHO_MSG(
		" Last Updated: " STRING_DISTRIBUTION_DATE
		" | Author: " STRING_CONFIG_H_AUTHOR
	);
#endif
	SERIAL_ECHO_MSG(" Compiled: " __DATE__);
	SERIAL_ECHO_MSG(STR_FREE_MEMORY, hal.freeMemory(), STR_PLANNER_BUFFER_BYTES, sizeof(block_t) * (BLOCK_BUFFER_SIZE));

	// Some HAL need precise delay adjustment
	calibrate_delay_loop();

	// Init buzzer pin(s)
#if HAS_BEEPER
	SETUP_RUN(buzzer.init());
#endif

	// Set up LEDs early
#if HAS_COLOR_LEDS
	SETUP_RUN(leds.setup());
#endif

#if ENABLED(NEOPIXEL2_SEPARATE)
	SETUP_RUN(leds2.setup());
#endif

#if ENABLED(USE_CONTROLLER_FAN)     // Set up fan controller to initialize also the default configurations.
	SETUP_RUN(controllerFan.setup());
#endif

	TERN_(HAS_FANCHECK, fan_check.init());

	// UI must be initialized before EEPROM
	// (because EEPROM code calls the UI).

	SETUP_RUN(ui.init());

#if PIN_EXISTS(SAFE_POWER)
#if HAS_DRIVER_SAFE_POWER_PROTECT
	SETUP_RUN(stepper_driver_backward_check());
#else
	SETUP_LOG("SAFE_POWER");
	OUT_WRITE(SAFE_POWER_PIN, HIGH);
#endif
#endif

#if BOTH(SDSUPPORT, SDCARD_EEPROM_EMULATION)
	SETUP_RUN(card.mount());          // Mount media with settings before first_load
#endif

	SETUP_RUN(settings.first_load());   // Load data from EEPROM if available (or use defaults)
										// This also updates variables in the planner, elsewhere

#if BOTH(HAS_WIRED_LCD, SHOW_BOOTSCREEN)
	SETUP_RUN(ui.show_bootscreen());
	const millis_t bootscreen_ms = millis();
#endif

#if ENABLED(PROBE_TARE)
	SETUP_RUN(probe.tare_init());
#endif

#if HAS_ETHERNET
	SETUP_RUN(ethernet.init());
#endif

#if HAS_TOUCH_BUTTONS
	SETUP_RUN(touchBt.init());
#endif

	TERN_(HAS_M206_COMMAND, current_position += home_offset); // Init current position based on home_offset

	sync_plan_position();               // Vital to init stepper/planner equivalent for current_position

	SETUP_RUN(thermalManager.init());   // Initialize temperature loop

	SETUP_RUN(print_job_timer.init());  // Initial setup of print job timer

	SETUP_RUN(endstops.init());         // Init endstops and pullups

#if ENABLED(DELTA) && !HAS_SOFTWARE_ENDSTOPS
	SETUP_RUN(refresh_delta_clip_start_height()); // Init safe delta height without soft endstops
#endif

	SETUP_RUN(stepper.init());          // Init stepper. This enables interrupts!

#if HAS_SERVOS
	SETUP_RUN(servo_init());
#endif

#if HAS_Z_SERVO_PROBE
	SETUP_RUN(probe.servo_probe_init());
#endif

#if HAS_PHOTOGRAPH
	OUT_WRITE(PHOTOGRAPH_PIN, LOW);
#endif

#if HAS_CUTTER
	SETUP_RUN(cutter.init());
#endif

#if ENABLED(COOLANT_MIST)
	OUT_WRITE(COOLANT_MIST_PIN, COOLANT_MIST_INVERT);   // Init Mist Coolant OFF
#endif
#if ENABLED(COOLANT_FLOOD)
	OUT_WRITE(COOLANT_FLOOD_PIN, COOLANT_FLOOD_INVERT); // Init Flood Coolant OFF
#endif

#if HAS_BED_PROBE
#if PIN_EXISTS(PROBE_ENABLE)
	OUT_WRITE(PROBE_ENABLE_PIN, LOW); // Disable
#endif
	SETUP_RUN(endstops.enable_z_probe(false));
#endif

#if HAS_STEPPER_RESET
	SETUP_RUN(enableStepperDrivers());
#endif

#if HAS_MOTOR_CURRENT_I2C
	SETUP_RUN(digipot_i2c.init());
#endif

#if HAS_MOTOR_CURRENT_DAC
	SETUP_RUN(stepper_dac.init());
#endif

#if EITHER(Z_PROBE_SLED, SOLENOID_PROBE) && HAS_SOLENOID_1
	OUT_WRITE(SOL1_PIN, LOW); // OFF
#endif

#if HAS_HOME
	SET_INPUT_PULLUP(HOME_PIN);
#endif

#if ENABLED(CUSTOM_USER_BUTTONS)
#define INIT_CUSTOM_USER_BUTTON_PIN(N) do{ SET_INPUT(BUTTON##N##_PIN); WRITE(BUTTON##N##_PIN, !BUTTON##N##_HIT_STATE); }while(0)

#if HAS_CUSTOM_USER_BUTTON(1)
	INIT_CUSTOM_USER_BUTTON_PIN(1);
#endif
#if HAS_CUSTOM_USER_BUTTON(2)
	INIT_CUSTOM_USER_BUTTON_PIN(2);
#endif
#if HAS_CUSTOM_USER_BUTTON(3)
	INIT_CUSTOM_USER_BUTTON_PIN(3);
#endif
#if HAS_CUSTOM_USER_BUTTON(4)
	INIT_CUSTOM_USER_BUTTON_PIN(4);
#endif
#if HAS_CUSTOM_USER_BUTTON(5)
	INIT_CUSTOM_USER_BUTTON_PIN(5);
#endif
#if HAS_CUSTOM_USER_BUTTON(6)
	INIT_CUSTOM_USER_BUTTON_PIN(6);
#endif
#if HAS_CUSTOM_USER_BUTTON(7)
	INIT_CUSTOM_USER_BUTTON_PIN(7);
#endif
#if HAS_CUSTOM_USER_BUTTON(8)
	INIT_CUSTOM_USER_BUTTON_PIN(8);
#endif
#if HAS_CUSTOM_USER_BUTTON(9)
	INIT_CUSTOM_USER_BUTTON_PIN(9);
#endif
#if HAS_CUSTOM_USER_BUTTON(10)
	INIT_CUSTOM_USER_BUTTON_PIN(10);
#endif
#if HAS_CUSTOM_USER_BUTTON(11)
	INIT_CUSTOM_USER_BUTTON_PIN(11);
#endif
#if HAS_CUSTOM_USER_BUTTON(12)
	INIT_CUSTOM_USER_BUTTON_PIN(12);
#endif
#if HAS_CUSTOM_USER_BUTTON(13)
	INIT_CUSTOM_USER_BUTTON_PIN(13);
#endif
#if HAS_CUSTOM_USER_BUTTON(14)
	INIT_CUSTOM_USER_BUTTON_PIN(14);
#endif
#if HAS_CUSTOM_USER_BUTTON(15)
	INIT_CUSTOM_USER_BUTTON_PIN(15);
#endif
#if HAS_CUSTOM_USER_BUTTON(16)
	INIT_CUSTOM_USER_BUTTON_PIN(16);
#endif
#if HAS_CUSTOM_USER_BUTTON(17)
	INIT_CUSTOM_USER_BUTTON_PIN(17);
#endif
#if HAS_CUSTOM_USER_BUTTON(18)
	INIT_CUSTOM_USER_BUTTON_PIN(18);
#endif
#if HAS_CUSTOM_USER_BUTTON(19)
	INIT_CUSTOM_USER_BUTTON_PIN(19);
#endif
#if HAS_CUSTOM_USER_BUTTON(20)
	INIT_CUSTOM_USER_BUTTON_PIN(20);
#endif
#if HAS_CUSTOM_USER_BUTTON(21)
	INIT_CUSTOM_USER_BUTTON_PIN(21);
#endif
#if HAS_CUSTOM_USER_BUTTON(22)
	INIT_CUSTOM_USER_BUTTON_PIN(22);
#endif
#if HAS_CUSTOM_USER_BUTTON(23)
	INIT_CUSTOM_USER_BUTTON_PIN(23);
#endif
#if HAS_CUSTOM_USER_BUTTON(24)
	INIT_CUSTOM_USER_BUTTON_PIN(24);
#endif
#if HAS_CUSTOM_USER_BUTTON(25)
	INIT_CUSTOM_USER_BUTTON_PIN(25);
#endif
#endif

#if PIN_EXISTS(STAT_LED_RED)
	OUT_WRITE(STAT_LED_RED_PIN, LOW); // OFF
#endif
#if PIN_EXISTS(STAT_LED_BLUE)
	OUT_WRITE(STAT_LED_BLUE_PIN, LOW); // OFF
#endif

#if ENABLED(CASE_LIGHT_ENABLE)
	SETUP_RUN(caselight.init());
#endif

#if HAS_PRUSA_MMU1
	SETUP_RUN(mmu_init());
#endif

#if HAS_FANMUX
	SETUP_RUN(fanmux_init());
#endif

#if ENABLED(MIXING_EXTRUDER)
	SETUP_RUN(mixer.init());
#endif

#if ENABLED(BLTOUCH)
	SETUP_RUN(bltouch.init(//set_voltage=true));
#endif

#if ENABLED(MAGLEV4)
	OUT_WRITE(MAGLEV_TRIGGER_PIN, LOW);
#endif

#if ENABLED(I2C_POSITION_ENCODERS)
	SETUP_RUN(I2CPEM.init());
#endif

#if ENABLED(EXPERIMENTAL_I2CBUS) && I2C_SLAVE_ADDRESS > 0
	SETUP_LOG("i2c...");
	i2c.onReceive(i2c_on_receive);
	i2c.onRequest(i2c_on_request);
#endif

#if DO_SWITCH_EXTRUDER
	SETUP_RUN(move_extruder_servo(0));  // Initialize extruder servo
#endif

#if ENABLED(SWITCHING_NOZZLE)
	SETUP_LOG("SWITCHING_NOZZLE");
	// Initialize nozzle servo(s)
#if SWITCHING_NOZZLE_TWO_SERVOS
	lower_nozzle(0);
	raise_nozzle(1);
#else
	move_nozzle_servo(0);
#endif
#endif

#if ENABLED(PARKING_EXTRUDER)
	SETUP_RUN(pe_solenoid_init());
#elif ENABLED(MAGNETIC_PARKING_EXTRUDER)
	SETUP_RUN(mpe_settings_init());
#elif ENABLED(SWITCHING_TOOLHEAD)
	SETUP_RUN(swt_init());
#elif ENABLED(ELECTROMAGNETIC_SWITCHING_TOOLHEAD)
	SETUP_RUN(est_init());
#endif

#if ENABLED(USE_WATCHDOG)
	SETUP_RUN(hal.watchdog_init());   // Reinit watchdog after hal.get_reset_source call
#endif

#if ENABLED(EXTERNAL_CLOSED_LOOP_CONTROLLER)
	SETUP_RUN(closedloop.init());
#endif

#ifdef STARTUP_COMMANDS
	SETUP_LOG("STARTUP_COMMANDS");
	queue.inject(F(STARTUP_COMMANDS));
#endif

#if ENABLED(HOST_PROMPT_SUPPORT)
	SETUP_RUN(hostui.prompt_end());
#endif

#if HAS_DRIVER_SAFE_POWER_PROTECT
	SETUP_RUN(stepper_driver_backward_report());
#endif

#if HAS_PRUSA_MMU2
	SETUP_RUN(mmu2.init());
#endif

#if ENABLED(IIC_BL24CXX_EEPROM)
	BL24CXX::init();
	const uint8_t err = BL24CXX::check();
	SERIAL_ECHO_TERNARY(err, "BL24CXX Check ", "failed", "succeeded", "!\n");
#endif

#if HAS_DWIN_E3V2_BASIC
	SETUP_RUN(DWIN_InitScreen());
#endif

#if HAS_SERVICE_INTERVALS && !HAS_DWIN_E3V2_BASIC
	SETUP_RUN(ui.reset_status(true));  // Show service messages or keep current status
#endif

#if ENABLED(MAX7219_DEBUG)
	SETUP_RUN(max7219.init());
#endif

#if ENABLED(DIRECT_STEPPING)
	SETUP_RUN(page_manager.init());
#endif

#if HAS_TFT_LVGL_UI
#if ENABLED(SDSUPPORT)
	if (!card.isMounted()) SETUP_RUN(card.mount()); // Mount SD to load graphics and fonts
#endif
	SETUP_RUN(tft_lvgl_init());
#endif

#if BOTH(HAS_WIRED_LCD, SHOW_BOOTSCREEN)
	const millis_t elapsed = millis() - bootscreen_ms;
#if ENABLED(MARLIN_DEV_MODE)
	SERIAL_ECHOLNPGM("elapsed=", elapsed);
#endif
	SETUP_RUN(ui.bootscreen_completion(elapsed));
#endif

#if ENABLED(PASSWORD_ON_STARTUP)
	SETUP_RUN(password.lock_machine());      // Will not proceed until correct password provided
#endif

#if BOTH(HAS_MARLINUI_MENU, TOUCH_SCREEN_CALIBRATION) && EITHER(TFT_CLASSIC_UI, TFT_COLOR_UI)
	SETUP_RUN(ui.check_touch_calibration());
#endif

#if ENABLED(EASYTHREED_UI)
	SETUP_RUN(easythreed_ui.init());
#endif

#if HAS_TRINAMIC_CONFIG && DISABLED(PSU_DEFAULT_OFF)
	SETUP_RUN(test_tmc_connection());
#endif

#if ENABLED(BD_SENSOR)
	SETUP_RUN(bdl.init(I2C_BD_SDA_PIN, I2C_BD_SCL_PIN, I2C_BD_DELAY));
#endif

	marlin_state = MF_RUNNING;

	SETUP_LOG("setup() completed.");

	TERN_(MARLIN_TEST_BUILD, runStartupTests());
*/
}//setup()
void loop(void) {

}//loop()

//MarlinCore.cpp
