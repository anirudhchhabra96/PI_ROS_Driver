// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the PI-Hexapod Driver.
//
//
// © Copyright 2020 Physik Instrumente (PI) GmbH & Co. KG, Karlsruhe, Germany
// © Copyright 2020 FZI Forschungszentrum Informatik, Karlsruhe, Germany
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * This file contains a map, matching error codes to error messages.
 *
 * \author  Christian Eichmann <eichmann@fzi.de>
 * \author  Philip Keller <keller@fzi.de>
 * \date    2020-05-25
 *
 */
//----------------------------------------------------------------------

#include <pi_hexapod_control/pi_errors.h>

namespace pi_hexapod_control {

std::unordered_map<int, std::string> g_PI_ERRORS_UMAP = {
  {PI_CNTR_NO_ERROR, "No error"},
  {PI_CNTR_PARAM_SYNTAX, "Parameter syntax error"},
  {PI_CNTR_UNKNOWN_COMMAND, "Unknown command"},
  {PI_CNTR_COMMAND_TOO_LONG, "Command length out of limits or command buffer overrun"},
  {PI_CNTR_SCAN_ERROR, "Error while scanning"},
  {PI_CNTR_MOVE_WITHOUT_REF_OR_NO_SERVO,
   "Unallowable move attempted on unreferenced axis, or move attempted with servo off"},
  {PI_CNTR_INVALID_SGA_PARAM, "Parameter for SGA not valid"},
  {PI_CNTR_POS_OUT_OF_LIMITS, "Position out of limits"},
  {PI_CNTR_VEL_OUT_OF_LIMITS, "Velocity out of limits"},
  {PI_CNTR_SET_PIVOT_NOT_POSSIBLE, "Attempt to set pivot point while U, V, and W not all 0"},
  {PI_CNTR_STOP, "Controller was stopped by command"},
  {PI_CNTR_SST_OR_SCAN_RANGE,
   "Parameter for SST or for one of the embedded scan algorithms out of range"},
  {PI_CNTR_INVALID_SCAN_AXES, "Invalid axis combination for fast scan"},
  {PI_CNTR_INVALID_NAV_PARAM, "Parameter for NAV out of range"},
  {PI_CNTR_INVALID_ANALOG_INPUT, "Invalid analog channel"},
  {PI_CNTR_INVALID_AXIS_IDENTIFIER, "Invalid axis identifier"},
  {PI_CNTR_INVALID_STAGE_NAME, "Unknown stage name"},
  {PI_CNTR_PARAM_OUT_OF_RANGE, "Parameter out of range"},
  {PI_CNTR_INVALID_MACRO_NAME, "Invalid macro name"},
  {PI_CNTR_MACRO_RECORD, "Error while recording macro"},
  {PI_CNTR_MACRO_NOT_FOUND, "Macro not found"},
  {PI_CNTR_AXIS_HAS_NO_BRAKE, "Axis has no brake"},
  {PI_CNTR_DOUBLE_AXIS, "Axis identifier specified more than once"},
  {PI_CNTR_ILLEGAL_AXIS, "Illegal axis"},
  {PI_CNTR_PARAM_NR, "Incorrect number of parameters"},
  {PI_CNTR_INVALID_REAL_NR, "Invalid floating point number"},
  {PI_CNTR_MISSING_PARAM, "Parameter missing"},
  {PI_CNTR_SOFT_LIMIT_OUT_OF_RANGE, "Soft limit out of range"},
  {PI_CNTR_NO_MANUAL_PAD, "No manual pad found"},
  {PI_CNTR_NO_JUMP, "No more step-response values"},
  {PI_CNTR_INVALID_JUMP, "No step-response values recorded"},
  {PI_CNTR_AXIS_HAS_NO_REFERENCE, "Axis has no reference sensor"},
  {PI_CNTR_STAGE_HAS_NO_LIM_SWITCH, "Axis has no limit switch"},
  {PI_CNTR_NO_RELAY_CARD, "No relay card installed"},
  {PI_CNTR_CMD_NOT_ALLOWED_FOR_STAGE, "Command not allowed for selected stage(s)"},
  {PI_CNTR_NO_DIGITAL_INPUT, "No digital input installed"},
  {PI_CNTR_NO_DIGITAL_OUTPUT, "No digital output configured"},
  {PI_CNTR_NO_MCM, "No more MCM responses"},
  {PI_CNTR_INVALID_MCM, "No MCM values recorded"},
  {PI_CNTR_INVALID_CNTR_NUMBER, "Controller number invalid"},
  {PI_CNTR_NO_JOYSTICK_CONNECTED, "No joystick configured"},
  {PI_CNTR_INVALID_EGE_AXIS, "Invalid axis for electronic gearing, axis cannot be slave"},
  {PI_CNTR_SLAVE_POSITION_OUT_OF_RANGE, "Position of slave axis is out of range"},
  {PI_CNTR_COMMAND_EGE_SLAVE,
   "Slave axis cannot be commanded directly when electronic gearing is enabled"},
  {PI_CNTR_JOYSTICK_CALIBRATION_FAILED, "Calibration of joystick failed"},
  {PI_CNTR_REFERENCING_FAILED, "Referencing failed"},
  {PI_CNTR_OPM_MISSING, "OPM (Optical Power Meter) missing"},
  {PI_CNTR_OPM_NOT_INITIALIZED,
   "OPM (Optical Power Meter) not initialized or cannot be initialized"},
  {PI_CNTR_OPM_COM_ERROR, "OPM (Optical Power Meter) communication error"},
  {PI_CNTR_MOVE_TO_LIMIT_SWITCH_FAILED, "Move to limit switch failed"},
  {PI_CNTR_REF_WITH_REF_DISABLED, "Attempt to reference axis with referencing disabled"},
  {PI_CNTR_AXIS_UNDER_JOYSTICK_CONTROL, "Selected axis is controlled by joystick"},
  {PI_CNTR_COMMUNICATION_ERROR, "Controller detected communication error"},
  {PI_CNTR_DYNAMIC_MOVE_IN_PROCESS, "MOV! motion still in progress"},
  {PI_CNTR_UNKNOWN_PARAMETER, "Unknown parameter"},
  {PI_CNTR_NO_REP_RECORDED, "No commands were recorded with REP"},
  {PI_CNTR_INVALID_PASSWORD, "Password invalid"},
  {PI_CNTR_INVALID_RECORDER_CHAN, "Data record table does not exist"},
  {PI_CNTR_INVALID_RECORDER_SRC_OPT, "Source does not exist; number too low or too high"},
  {PI_CNTR_INVALID_RECORDER_SRC_CHAN, "Source record table number too low or too high"},
  {PI_CNTR_PARAM_PROTECTION, "Protected Param: Current Command Level (CCL) too low"},
  {PI_CNTR_AUTOZERO_RUNNING, "Command execution not possible while autozero is running"},
  {PI_CNTR_NO_LINEAR_AXIS, "Autozero requires at least one linear axis"},
  {PI_CNTR_INIT_RUNNING, "Initialization still in progress"},
  {PI_CNTR_READ_ONLY_PARAMETER, "Parameter is read-only"},
  {PI_CNTR_PAM_NOT_FOUND, "Parameter not found in nonvolatile memory"},
  {PI_CNTR_VOL_OUT_OF_LIMITS, "Voltage out of limits"},
  {PI_CNTR_WAVE_TOO_LARGE, "Not enough memory available for requested wave curve"},
  {PI_CNTR_NOT_ENOUGH_DDL_MEMORY,
   "Not enough memory available for DDL table; DDL cannot be started"},
  {PI_CNTR_DDL_TIME_DELAY_TOO_LARGE, "Time delay larger than DDL table; DDL cannot be started"},
  {PI_CNTR_DIFFERENT_ARRAY_LENGTH,
   "The requested arrays have different lengths; query them separately"},
  {PI_CNTR_GEN_SINGLE_MODE_RESTART,
   "Attempt to restart the generator while it is running in single step mode"},
  {PI_CNTR_ANALOG_TARGET_ACTIVE,
   "Motion commands and wave generator activation are not allowed when analog target is active"},
  {PI_CNTR_WAVE_GENERATOR_ACTIVE, "Motion commands are not allowed when wave generator is active"},
  {PI_CNTR_AUTOZERO_DISABLED,
   "No sensor channel or no piezo channel connected to selected axis (sensor and piezo matrix)"},
  {PI_CNTR_NO_WAVE_SELECTED, "Generator started (WGO) without having selected a wave table (WSL)."},
  {PI_CNTR_IF_BUFFER_OVERRUN,
   "Interface buffer overran and command couldn't be received correctly"},
  {PI_CNTR_NOT_ENOUGH_RECORDED_DATA, "Data record table does not hold enough recorded data"},
  {PI_CNTR_TABLE_DEACTIVATED, "Data record table is not configured for recording"},
  {PI_CNTR_OPENLOOP_VALUE_SET_WHEN_SERVO_ON,
   "Open-loop commands (SVA, SVR) are not allowed when servo is on"},
  {PI_CNTR_RAM_ERROR, "Hardware error affecting RAM"},
  {PI_CNTR_MACRO_UNKNOWN_COMMAND, "Not macro command"},
  {PI_CNTR_MACRO_PC_ERROR, "Macro counter out of range"},
  {PI_CNTR_JOYSTICK_ACTIVE, "Joystick is active"},
  {PI_CNTR_MOTOR_IS_OFF, "Motor is off"},
  {PI_CNTR_ONLY_IN_MACRO, "Macro-only command"},
  {PI_CNTR_JOYSTICK_UNKNOWN_AXIS, "Invalid joystick axis"},
  {PI_CNTR_JOYSTICK_UNKNOWN_ID, "Joystick unknown"},
  {PI_CNTR_REF_MODE_IS_ON, "Move without referenced stage"},
  {PI_CNTR_NOT_ALLOWED_IN_CURRENT_MOTION_MODE, "Command not allowed in current motion mode"},
  {PI_CNTR_DIO_AND_TRACING_NOT_POSSIBLE,
   "No tracing possible while digital IOs are used on this HW revision. Reconnect to switch "
   "operation mode."},
  {PI_CNTR_COLLISION, "Move not possible, would cause collision"},
  {PI_CNTR_SLAVE_NOT_FAST_ENOUGH,
   "Stage is not capable of following the master. Check the gear ratio."},
  {PI_CNTR_CMD_NOT_ALLOWED_WHILE_AXIS_IN_MOTION,
   "This command is not allowed while the affected axis or its master is in motion."},
  {PI_CNTR_OPEN_LOOP_JOYSTICK_ENABLED,
   "Servo cannot be switched on when open-loop joystick control is enabled."},
  {PI_CNTR_INVALID_SERVO_STATE_FOR_PARAMETER,
   "This parameter cannot be changed in current servo mode."},
  {PI_CNTR_UNKNOWN_STAGE_NAME, "Unknown stage name"},
  {PI_CNTR_INVALID_VALUE_LENGTH, "Invalid length of value (too much characters)"},
  {PI_CNTR_AUTOZERO_FAILED, "AutoZero procedure was not successful"},
  {PI_CNTR_SENSOR_VOLTAGE_OFF, "Sensor voltage is off"},
  {PI_LABVIEW_ERROR, "PI LabVIEW driver reports error. See source control for details."},
  {PI_CNTR_NO_AXIS, "No stage connected to axis"},
  {PI_CNTR_NO_AXIS_PARAM_FILE, "File with axis parameters not found"},
  {PI_CNTR_INVALID_AXIS_PARAM_FILE, "Invalid axis parameter file"},
  {PI_CNTR_NO_AXIS_PARAM_BACKUP, "Backup file with axis parameters not found"},
  {PI_CNTR_RESERVED_204, "PI internal error code 204"},
  {PI_CNTR_SMO_WITH_SERVO_ON, "SMO with servo on"},
  {PI_CNTR_UUDECODE_INCOMPLETE_HEADER, "uudecode: incomplete header"},
  {PI_CNTR_UUDECODE_NOTHING_TO_DECODE, "uudecode: nothing to decode"},
  {PI_CNTR_UUDECODE_ILLEGAL_FORMAT, "uudecode: illegal UUE format"},
  {PI_CNTR_CRC32_ERROR, "CRC32 error"},
  {PI_CNTR_ILLEGAL_FILENAME, "Illegal file name (must be 8-0 format)"},
  {PI_CNTR_FILE_NOT_FOUND, "File not found on controller"},
  {PI_CNTR_FILE_WRITE_ERROR, "Error writing file on controller"},
  {PI_CNTR_DTR_HINDERS_VELOCITY_CHANGE, "VEL command not allowed in DTR command mode"},
  {PI_CNTR_POSITION_UNKNOWN, "Position calculations failed"},
  {PI_CNTR_CONN_POSSIBLY_BROKEN, "The connection between controller and stage may be broken"},
  {PI_CNTR_ON_LIMIT_SWITCH,
   "The connected stage has driven into a limit switch, some controllers need CLR to resume "
   "operation"},
  {PI_CNTR_UNEXPECTED_STRUT_STOP, "Strut test command failed because of an unexpected strut stop"},
  {PI_CNTR_POSITION_BASED_ON_ESTIMATION, "While MOV! is running position can only be estimated!"},
  {PI_CNTR_POSITION_BASED_ON_INTERPOLATION, "Position was calculated during MOV motion"},
  {PI_CNTR_INTERPOLATION_FIFO_UNDERRUN, "FIFO buffer underrun during interpolation"},
  {PI_CNTR_INTERPOLATION_FIFO_OVERFLOW, "FIFO buffer underrun during interpolation"},
  {PI_CNTR_INVALID_HANDLE, "Invalid handle"},
  {PI_CNTR_NO_BIOS_FOUND, "No bios found"},
  {PI_CNTR_SAVE_SYS_CFG_FAILED, "Save system configuration failed"},
  {PI_CNTR_LOAD_SYS_CFG_FAILED, "Load system configuration failed"},
  {PI_CNTR_SEND_BUFFER_OVERFLOW, "Send buffer overflow"},
  {PI_CNTR_VOLTAGE_OUT_OF_LIMITS, "Voltage out of limits"},
  {PI_CNTR_OPEN_LOOP_MOTION_SET_WHEN_SERVO_ON, "Open-loop motion attempted when servo ON"},
  {PI_CNTR_RECEIVING_BUFFER_OVERFLOW, "Received command is too long"},
  {PI_CNTR_EEPROM_ERROR, "Error while reading/writing EEPROM"},
  {PI_CNTR_I2C_ERROR, "Error on I2C bus"},
  {PI_CNTR_RECEIVING_TIMEOUT, "Timeout while receiving command"},
  {PI_CNTR_TIMEOUT, "A lengthy operation has not finished in the expected time"},
  {PI_CNTR_MACRO_OUT_OF_SPACE, "Insufficient space to store macro"},
  {PI_CNTR_EUI_OLDVERSION_CFGDATA, "Configuration data has old version number"},
  {PI_CNTR_EUI_INVALID_CFGDATA, "Invalid configuration data"},
  {PI_CNTR_HARDWARE_ERROR, "Internal hardware error"},
  {PI_CNTR_WAV_INDEX_ERROR, "Wave generator index error"},
  {PI_CNTR_WAV_NOT_DEFINED, "Wave table not defined"},
  {PI_CNTR_WAV_TYPE_NOT_SUPPORTED, "Wave type not supported"},
  {PI_CNTR_WAV_LENGTH_EXCEEDS_LIMIT, "Wave length exceeds limit"},
  {PI_CNTR_WAV_PARAMETER_NR, "Wave parameter number error"},
  {PI_CNTR_WAV_PARAMETER_OUT_OF_LIMIT, "Wave parameter out of range"},
  {PI_CNTR_WGO_BIT_NOT_SUPPORTED, "WGO command bit not supported"},
  {PI_CNTR_EMERGENCY_STOP_BUTTON_ACTIVATED, "The \"red knob\" is still set and disables system"},
  {PI_CNTR_EMERGENCY_STOP_BUTTON_WAS_ACTIVATED,
   "The \"red knob\" was activated and still disables system - reanimation required"},
  {PI_CNTR_REDUNDANCY_LIMIT_EXCEEDED, "Position consistency check failed"},
  {PI_CNTR_COLLISION_SWITCH_ACTIVATED, "Hardware collision sensor(s) are activated"},
  {PI_CNTR_FOLLOWING_ERROR,
   "Strut following error occurred, e.g., caused by overload or encoder failure"},
  {PI_CNTR_SENSOR_SIGNAL_INVALID, "One sensor signal is not valid"},
  {PI_CNTR_SERVO_LOOP_UNSTABLE,
   "Servo loop was unstable due to wrong parameter setting and switched off to avoid damage."},
  {PI_CNTR_LOST_SPI_SLAVE_CONNECTION, "Digital connection to external SPI slave device is lost"},
  {PI_CNTR_NODE_DOES_NOT_EXIST, "A command refers to a node that does not exist"},
  {PI_CNTR_PARENT_NODE_DOES_NOT_EXIST, "A command refers to a node that has no parent node"},
  {PI_CNTR_NODE_IN_USE, "Attempt to delete a node that is in use"},
  {PI_CNTR_NODE_DEFINITION_IS_CYCLIC, "Definition of a node is cyclic"},
  {PI_CNTR_HEXAPOD_IN_MOTION, "Transformation cannot be defined as long as Hexapod is in motion"},
  {PI_CNTR_TRANSFORMATION_TYPE_NOT_SUPPORTED, "Transformation node cannot be activated"},
  {PI_CNTR_NODE_PARENT_IDENTICAL_TO_CHILD, "A node cannot be linked to itself"},
  {PI_CNTR_NODE_DEFINITION_INCONSISTENT,
   "Node definition is erroneous or not complete (replace or delete it)"},
  {PI_CNTR_NODES_NOT_IN_SAME_CHAIN, "The nodes are not part of the same chain"},
  {PI_CNTR_NODE_MEMORY_FULL, "Unused nodes must be deleted before new nodes can be stored"},
  {PI_CNTR_PIVOT_POINT_FEATURE_NOT_SUPPORTED,
   "With some transformations pivot point usage is not supported"},
  {PI_CNTR_SOFTLIMITS_INVALID, "Soft limits invalid due to changes in coordinate system"},
  {PI_CNTR_CS_WRITE_PROTECTED, "Coordinate system is write protected"},
  {PI_CNTR_CS_CONTENT_FROM_CONFIG_FILE,
   "Coordinate system cannot be changed because its content is loaded from a configuration file"},
  {PI_CNTR_CS_CANNOT_BE_LINKED, "Coordinate system may not be linked"},
  {PI_CNTR_KSB_CS_ROTATION_ONLY,
   "A KSB-type coordinate system can only be rotated by multiples of 90 degrees"},
  {PI_CNTR_CS_DATA_CANNOT_BE_QUERIED,
   "This query is not supported for this coordinate system type"},
  {PI_CNTR_CS_COMBINATION_DOES_NOT_EXIST,
   "This combination of work and tool coordinate systems does not exist"},
  {PI_CNTR_CS_COMBINATION_INVALID,
   "The combination must consist of one work and one tool coordinate system"},
  {PI_CNTR_CS_TYPE_DOES_NOT_EXIST, "This coordinate system type does not exist"},
  {PI_CNTR_UNKNOWN_ERROR, "BasMac: unknown controller error"},
  {PI_CNTR_CS_TYPE_NOT_ACTIVATED, "No coordinate system of this type is activated"},
  {PI_CNTR_CS_NAME_INVALID, "Name of coordinate system is invalid"},
  {PI_CNTR_CS_GENERAL_FILE_MISSING, "File with stored CS systems is missing or erroneous"},
  {PI_CNTR_CS_LEVELING_FILE_MISSING, "File with leveling CS is missing or erroneous"},
  {PI_CNTR_NOT_ENOUGH_MEMORY, "Not enough memory"},
  {PI_CNTR_HW_VOLTAGE_ERROR, "Hardware voltage error"},
  {PI_CNTR_HW_TEMPERATURE_ERROR, "Hardware temperature out of range"},
  {PI_CNTR_POSITION_ERROR_TOO_HIGH, "Position error of any axis in the system is too high"},
  {PI_CNTR_INPUT_OUT_OF_RANGE, "Maximum value of input signal has been exceeded"},
  {PI_CNTR_NO_INTEGER, "Value is not integer"},
  {PI_CNTR_FAST_ALIGNMENT_PROCESS_IS_NOT_RUNNING,
   "Fast alignment process cannot be paused because it is not running"},
  {PI_CNTR_FAST_ALIGNMENT_PROCESS_IS_NOT_PAUSED,
   "Fast alignment process cannot be restarted/resumed because it is not paused"},
  {PI_CNTR_UNABLE_TO_SET_PARAM_WITH_SPA, "Parameter could not be set with SPA - SEP needed?"},
  {PI_CNTR_PHASE_FINDING_ERROR, "Phase finding error"},
  {PI_CNTR_SENSOR_SETUP_ERROR, "Sensor setup error"},
  {PI_CNTR_SENSOR_COMM_ERROR, "Sensor communication error"},
  {PI_CNTR_MOTOR_AMPLIFIER_ERROR, "Motor amplifier error"},
  {PI_CNTR_OVER_CURR_PROTEC_TRIGGERED_BY_I2T, "Overcurrent protection triggered by I2T-module"},
  {PI_CNTR_OVER_CURR_PROTEC_TRIGGERED_BY_AMP_MODULE,
   "Overcurrent protection triggered by amplifier module"},
  {PI_CNTR_SAFETY_STOP_TRIGGERED, "Safety stop triggered"},
  {PI_SENSOR_OFF, "Sensor off?"},
  {PI_CNTR_COMMAND_NOT_ALLOWED_IN_EXTERNAL_MODE, "Command not allowed in external mode"},
  {PI_CNTR_EXTERNAL_MODE_ERROR, "External mode communication error"},
  {PI_CNTR_INVALID_MODE_OF_OPERATION, "Invalid mode of operation"},
  {PI_CNTR_FIRMWARE_STOPPED_BY_CMD, "Firmware stopped by command (#27)"},
  {PI_CNTR_EXTERNAL_MODE_DRIVER_MISSING, "External mode driver missing"},
  {PI_CNTR_CONFIGURATION_FAILURE_EXTERNAL_MODE,
   "Missing or incorrect configuration of external mode"},
  {PI_CNTR_EXTERNAL_MODE_CYCLETIME_INVALID, "External mode cycle time invalid"},
  {PI_CNTR_BRAKE_ACTIVATED, "Brake is activated"},
  {PI_CNTR_SURFACEDETECTION_RUNNING, "Command not allowed while surface detection is running"},
  {PI_CNTR_SURFACEDETECTION_FAILED, "Last surface detection failed"},
  {PI_CNTR_FIELDBUS_IS_ACTIVE, "Fieldbus is active and is blocking GCS controlcommands"},
  {PI_CNTR_TOO_MANY_NESTED_MACROS, "Too many nested macros"},
  {PI_CNTR_MACRO_ALREADY_DEFINED, "Macro already defined"},
  {PI_CNTR_NO_MACRO_RECORDING, "Macro recording not activated"},
  {PI_CNTR_INVALID_MAC_PARAM, "Invalid parameter for MAC"},
  {PI_CNTR_RESERVED_1004, "PI internal error code 1004"},
  {PI_CNTR_CONTROLLER_BUSY,
   "Controller is busy with some lengthy operation (e.g., reference move, fast scan algorithm)"},
  {PI_CNTR_INVALID_IDENTIFIER, "Invalid identifier (invalid special characters, ...)"},
  {PI_CNTR_UNKNOWN_VARIABLE_OR_ARGUMENT, "Variable or argument not defined"},
  {PI_CNTR_RUNNING_MACRO, "Controller is (already) running a macro"},
  {PI_CNTR_MACRO_INVALID_OPERATOR,
   "Invalid or missing operator for condition. Check necessary spaces around operator."},
  {PI_CNTR_MACRO_NO_ANSWER, "No answer was received while executing WAC/MEX/JRC/..."},
  {PI_CMD_NOT_VALID_IN_MACRO_MODE, "Command not valid during macro execution"},
  {PI_CNTR_MOTION_ERROR,
   "Motion error: position error too large, servo is switched off automatically"},
  {PI_CNTR_MAX_MOTOR_OUTPUT_REACHED, "Maximum motor output reached"},
  {PI_CNTR_EXT_PROFILE_UNALLOWED_CMD,
   "User profile mode: command is not allowed, check for required preparatory commands"},
  {PI_CNTR_EXT_PROFILE_EXPECTING_MOTION_ERROR,
   "User profile mode: first target position in user profile is too far from current position"},
  {PI_CNTR_PROFILE_ACTIVE, "Controller is (already) in user profile mode"},
  {PI_CNTR_PROFILE_INDEX_OUT_OF_RANGE,
   "User profile mode: block or data set index out of allowed range"},
  {PI_CNTR_PROFILE_OUT_OF_MEMORY, "User profile mode: out of memory"},
  {PI_CNTR_PROFILE_WRONG_CLUSTER, "User profile mode: cluster is not assigned to this axis"},
  {PI_CNTR_PROFILE_UNKNOWN_CLUSTER_IDENTIFIER, "Unknown cluster identifier"},
  {PI_CNTR_TOO_MANY_TCP_CONNECTIONS_OPEN, "There are too many open tcpip connections"},
  {PI_CNTR_ALREADY_HAS_SERIAL_NUMBER, "Controller already has a serial number"},
  {PI_CNTR_SECTOR_ERASE_FAILED, "Sector erase failed"},
  {PI_CNTR_FLASH_PROGRAM_FAILED, "Flash program failed"},
  {PI_CNTR_FLASH_READ_FAILED, "Flash read failed"},
  {PI_CNTR_HW_MATCHCODE_ERROR, "HW match code missing/invalid"},
  {PI_CNTR_FW_MATCHCODE_ERROR, "FW match code missing/invalid"},
  {PI_CNTR_HW_VERSION_ERROR, "HW version missing/invalid"},
  {PI_CNTR_FW_VERSION_ERROR, "FW version missing/invalid"},
  {PI_CNTR_FW_UPDATE_ERROR, "FW update failed"},
  {PI_CNTR_FW_CRC_PAR_ERROR, "FW Parameter CRC wrong"},
  {PI_CNTR_FW_CRC_FW_ERROR, "FW CRC wrong"},
  {PI_CNTR_INVALID_PCC_SCAN_DATA, "PicoCompensation scan data is not valid"},
  {PI_CNTR_PCC_SCAN_RUNNING,
   "PicoCompensation is running, some actions cannot be performed during scanning/recording"},
  {PI_CNTR_INVALID_PCC_AXIS, "Given axis can not be defined as PPC axis"},
  {PI_CNTR_PCC_SCAN_OUT_OF_RANGE, "Defined scan area is larger than the travel range"},
  {PI_CNTR_PCC_TYPE_NOT_EXISTING, "Given PicoCompensation type is not defined"},
  {PI_CNTR_PCC_PAM_ERROR, "PicoCompensation parameter error"},
  {PI_CNTR_PCC_TABLE_ARRAY_TOO_LARGE, "PicoCompensation table is larger than maximum table length"},
  {PI_CNTR_NEXLINE_ERROR, "Common error in NEXLINE® firmware module"},
  {PI_CNTR_CHANNEL_ALREADY_USED, "Output channel for NEXLINE® cannot be redefined for other usage"},
  {PI_CNTR_NEXLINE_TABLE_TOO_SMALL, "Memory for NEXLINE® signals is too small"},
  {PI_CNTR_RNP_WITH_SERVO_ON, "RNP cannot be executed if axis is in closed loop"},
  {PI_CNTR_RNP_NEEDED, "Relax procedure (RNP) needed"},
  {PI_CNTR_AXIS_NOT_CONFIGURED, "Axis must be configured for this action"},
  {PI_CNTR_FREQU_ANALYSIS_FAILED, "Frequency analysis failed"},
  {PI_CNTR_FREQU_ANALYSIS_RUNNING, "Another frequency analysis is running"},
  {PI_CNTR_SENSOR_ABS_INVALID_VALUE, "Invalid preset value of absolute sensor"},
  {PI_CNTR_SENSOR_ABS_WRITE_ERROR, "Error while writing to sensor"},
  {PI_CNTR_SENSOR_ABS_READ_ERROR, "Error while reading from sensor"},
  {PI_CNTR_SENSOR_ABS_CRC_ERROR, "Checksum error of absolute sensor"},
  {PI_CNTR_SENSOR_ABS_ERROR, "General error of absolute sensor"},
  {PI_CNTR_SENSOR_ABS_OVERFLOW, "Overflow of absolute sensor position"},
  {COM_ERROR, "Error during com operation (could not be specified)"},
  {SEND_ERROR, "Error while sending data"},
  {REC_ERROR, "Error while receiving data"},
  {NOT_CONNECTED_ERROR, "Not connected (no port with given ID open)"},
  {COM_BUFFER_OVERFLOW, "Buffer overflow"},
  {CONNECTION_FAILED, "Error while opening port"},
  {COM_TIMEOUT, "Timeout error"},
  {COM_MULTILINE_RESPONSE, "There are more lines waiting in buffer"},
  {COM_INVALID_ID, "There is no interface or DLL handle with the given ID"},
  {COM_NOTIFY_EVENT_ERROR, "Event/message for notification could not be opened"},
  {COM_NOT_IMPLEMENTED, "Function not supported by this interface type"},
  {COM_ECHO_ERROR, "Error while sending \"echoed\" data"},
  {COM_GPIB_EDVR, "IEEE488: System error"},
  {COM_GPIB_ECIC, "IEEE488: Function requires GPIB board to be CIC"},
  {COM_GPIB_ENOL, "IEEE488: Write function detected no listeners"},
  {COM_GPIB_EADR, "IEEE488: Interface board not addressed correctly"},
  {COM_GPIB_EARG, "IEEE488: Invalid argument to function call"},
  {COM_GPIB_ESAC, "IEEE488: Function requires GPIB board to be SAC"},
  {COM_GPIB_EABO, "IEEE488: I/O operation aborted"},
  {COM_GPIB_ENEB, "IEEE488: Interface board not found"},
  {COM_GPIB_EDMA, "IEEE488: Error performing DMA"},
  {COM_GPIB_EOIP, "IEEE488: I/O operation started before previous operation completed"},
  {COM_GPIB_ECAP, "IEEE488: No capability for intended operation"},
  {COM_GPIB_EFSO, "IEEE488: File system operation error"},
  {COM_GPIB_EBUS, "IEEE488: Command error during device call"},
  {COM_GPIB_ESTB, "IEEE488: Serial poll-status byte lost"},
  {COM_GPIB_ESRQ, "IEEE488: SRQ remains asserted"},
  {COM_GPIB_ETAB, "IEEE488: Return buffer full"},
  {COM_GPIB_ELCK, "IEEE488: Address or board locked"},
  {COM_RS_INVALID_DATA_BITS,
   "RS-232: 5 data bits with 2 stop bits is an invalid combination, as is 6, 7, or 8 data bits "
   "with 1.5 stop bits"},
  {COM_ERROR_RS_SETTINGS, "RS-232: Error configuring the COM port"},
  {COM_INTERNAL_RESOURCES_ERROR,
   "Error dealing with internal system resources (events, threads, ...)"},
  {COM_DLL_FUNC_ERROR, "A DLL or one of the required functions could not be loaded"},
  {COM_FTDIUSB_INVALID_HANDLE, "FTDIUSB: invalid handle"},
  {COM_FTDIUSB_DEVICE_NOT_FOUND, "FTDIUSB: device not found"},
  {COM_FTDIUSB_DEVICE_NOT_OPENED, "FTDIUSB: device not opened"},
  {COM_FTDIUSB_IO_ERROR, "FTDIUSB: IO error"},
  {COM_FTDIUSB_INSUFFICIENT_RESOURCES, "FTDIUSB: insufficient resources"},
  {COM_FTDIUSB_INVALID_PARAMETER, "FTDIUSB: invalid parameter"},
  {COM_FTDIUSB_INVALID_BAUD_RATE, "FTDIUSB: invalid baud rate"},
  {COM_FTDIUSB_DEVICE_NOT_OPENED_FOR_ERASE, "FTDIUSB: device not opened for erase"},
  {COM_FTDIUSB_DEVICE_NOT_OPENED_FOR_WRITE, "FTDIUSB: device not opened for write"},
  {COM_FTDIUSB_FAILED_TO_WRITE_DEVICE, "FTDIUSB: failed to write device"},
  {COM_FTDIUSB_EEPROM_READ_FAILED, "FTDIUSB: EEPROM read failed"},
  {COM_FTDIUSB_EEPROM_WRITE_FAILED, "FTDIUSB: EEPROM write failed"},
  {COM_FTDIUSB_EEPROM_ERASE_FAILED, "FTDIUSB: EEPROM erase failed"},
  {COM_FTDIUSB_EEPROM_NOT_PRESENT, "FTDIUSB: EEPROM not present"},
  {COM_FTDIUSB_EEPROM_NOT_PROGRAMMED, "FTDIUSB: EEPROM not programmed"},
  {COM_FTDIUSB_INVALID_ARGS, "FTDIUSB: invalid arguments"},
  {COM_FTDIUSB_NOT_SUPPORTED, "FTDIUSB: not supported"},
  {COM_FTDIUSB_OTHER_ERROR, "FTDIUSB: other error"},
  {COM_PORT_ALREADY_OPEN, "Error while opening the COM port: was already open"},
  {COM_PORT_CHECKSUM_ERROR, "Checksum error in received data from COM port"},
  {COM_SOCKET_NOT_READY, "Socket not ready, you should call the function again"},
  {COM_SOCKET_PORT_IN_USE, "Port is used by another socket"},
  {COM_SOCKET_NOT_CONNECTED, "Socket not connected (or not valid)"},
  {COM_SOCKET_TERMINATED, "Connection terminated (by peer)"},
  {COM_SOCKET_NO_RESPONSE, "Can't connect to peer"},
  {COM_SOCKET_INTERRUPTED, "Operation was interrupted by a nonblocked signal"},
  {COM_PCI_INVALID_ID, "No device with this ID is present"},
  {COM_PCI_ACCESS_DENIED, "Driver could not be opened (on Vista: run as administrator!)"},
  {COM_SOCKET_HOST_NOT_FOUND, "Host not found"},
  {COM_DEVICE_CONNECTED, "Device already connected"},
  {PI_UNKNOWN_AXIS_IDENTIFIER, "Unknown axis identifier"},
  {PI_NR_NAV_OUT_OF_RANGE, "Number for NAV out of range--must be in [1.10000]"},
  {PI_INVALID_SGA, "Invalid value for SGA--must be one of 1, 10, 100, 1000"},
  {PI_UNEXPECTED_RESPONSE, "Controller sent unexpected response"},
  {PI_NO_MANUAL_PAD,
   "No manual control pad installed, calls to SMA and related commands are not allowed"},
  {PI_INVALID_MANUAL_PAD_KNOB, "Invalid number for manual control pad knob"},
  {PI_INVALID_MANUAL_PAD_AXIS, "Axis not currently controlled by a manual control pad"},
  {PI_CONTROLLER_BUSY,
   "Controller is busy with some lengthy operation (e.g., reference move, fast scan algorithm)"},
  {PI_THREAD_ERROR, "Internal error--could not start thread"},
  {PI_IN_MACRO_MODE, "Controller is (already) in macro mode--command not valid in macro mode"},
  {PI_NOT_IN_MACRO_MODE,
   "Controller not in macro mode--command not valid unless macro mode active"},
  {PI_MACRO_FILE_ERROR, "Could not open file to write or read macro"},
  {PI_NO_MACRO_OR_EMPTY, "No macro with given name on controller, or macro is empty"},
  {PI_MACRO_EDITOR_ERROR, "Internal error in macro editor"},
  {PI_INVALID_ARGUMENT,
   "One or more arguments given to function is invalid (empty string, index out of range, ...)"},
  {PI_AXIS_ALREADY_EXISTS, "Axis identifier is already in use by a connected stage"},
  {PI_INVALID_AXIS_IDENTIFIER, "Invalid axis identifier"},
  {PI_COM_ARRAY_ERROR, "Could not access array data in COM server"},
  {PI_COM_ARRAY_RANGE_ERROR, "Range of array does not fit the number of parameters"},
  {PI_INVALID_SPA_CMD_ID, "Invalid parameter ID given to SPA or SPA?"},
  {PI_NR_AVG_OUT_OF_RANGE, "Number for AVG out of range--must be >0"},
  {PI_WAV_SAMPLES_OUT_OF_RANGE, "Incorrect number of samples given to WAV"},
  {PI_WAV_FAILED, "Generation of wave failed"},
  {PI_MOTION_ERROR, "Motion error: position error too large, servo is switched off automatically"},
  {PI_RUNNING_MACRO, "Controller is (already) running a macro"},
  {PI_PZT_CONFIG_FAILED, "Configuration of PZT stage or amplifier failed"},
  {PI_PZT_CONFIG_INVALID_PARAMS, "Current settings are not valid for desired configuration"},
  {PI_UNKNOWN_CHANNEL_IDENTIFIER, "Unknown channel identifier"},
  {PI_WAVE_PARAM_FILE_ERROR, "Error while reading/writing wave generator parameter file"},
  {PI_UNKNOWN_WAVE_SET, "Could not find description of wave form. Maybe WG.INI is missing?"},
  {PI_WAVE_EDITOR_FUNC_NOT_LOADED, "The WGWaveEditor DLL function was not found at startup"},
  {PI_USER_CANCELLED, "The user cancelled a dialog"},
  {PI_C844_ERROR, "Error from C-844 Controller"},
  {PI_DLL_NOT_LOADED, "DLL necessary to call function not loaded, or function not found in DLL"},
  {PI_PARAMETER_FILE_PROTECTED, "The open parameter file is protected and cannot be edited"},
  {PI_NO_PARAMETER_FILE_OPENED, "There is no parameter file open"},
  {PI_STAGE_DOES_NOT_EXIST, "Selected stage does not exist"},
  {PI_PARAMETER_FILE_ALREADY_OPENED,
   "There is already a parameter file open. Close it before opening a new file"},
  {PI_PARAMETER_FILE_OPEN_ERROR, "Could not open parameter file"},
  {PI_INVALID_CONTROLLER_VERSION, "The version of the connected controller is invalid"},
  {PI_PARAM_SET_ERROR,
   "Parameter could not be set with SPA-- parameter not defined for this controller!"},
  {PI_NUMBER_OF_POSSIBLE_WAVES_EXCEEDED,
   "The maximum number of wave definitions has been exceeded"},
  {PI_NUMBER_OF_POSSIBLE_GENERATORS_EXCEEDED,
   "The maximum number of wave generators has been exceeded"},
  {PI_NO_WAVE_FOR_AXIS_DEFINED, "No wave defined for specified axis"},
  {PI_CANT_STOP_OR_START_WAV, "Wave output to axis already stopped/started"},
  {PI_REFERENCE_ERROR, "Not all axes could be referenced"},
  {PI_REQUIRED_WAVE_NOT_FOUND, "Could not find parameter set required by frequency relation"},
  {PI_INVALID_SPP_CMD_ID, "Command ID given to SPP or SPP? is not valid"},
  {PI_STAGE_NAME_ISNT_UNIQUE, "A stage name given to CST is not unique"},
  {PI_FILE_TRANSFER_BEGIN_MISSING,
   "A uuencoded file transferred did not start with \"begin\" followed by the proper filename"},
  {PI_FILE_TRANSFER_ERROR_TEMP_FILE, "Could not create/read file on host PC"},
  {PI_FILE_TRANSFER_CRC_ERROR, "Checksum error when transferring a file to/from the controller"},
  {PI_COULDNT_FIND_PISTAGES_DAT,
   "The PiStages.dat database could not be found. This file is required to connect a stage with "
   "the CST command"},
  {PI_NO_WAVE_RUNNING, "No wave being output to specified axis"},
  {PI_INVALID_PASSWORD, "Invalid password"},
  {PI_OPM_COM_ERROR,
   "Error during communication with OPM (Optical Power Meter), maybe no OPM connected"},
  {PI_WAVE_EDITOR_WRONG_PARAMNUM,
   "WaveEditor: Error during wave creation, incorrect number of parameters"},
  {PI_WAVE_EDITOR_FREQUENCY_OUT_OF_RANGE, "WaveEditor: Frequency out of range"},
  {PI_WAVE_EDITOR_WRONG_IP_VALUE,
   "WaveEditor: Error during wave creation, incorrect index for integer parameter"},
  {PI_WAVE_EDITOR_WRONG_DP_VALUE,
   "WaveEditor: Error during wave creation, incorrect index for floating point parameter"},
  {PI_WAVE_EDITOR_WRONG_ITEM_VALUE,
   "WaveEditor: Error during wave creation, could not calculate value"},
  {PI_WAVE_EDITOR_MISSING_GRAPH_COMPONENT, "WaveEditor: Graph display component not installed"},
  {PI_EXT_PROFILE_UNALLOWED_CMD,
   "User profile mode: command is not allowed, check for required preparatory commands"},
  {PI_EXT_PROFILE_EXPECTING_MOTION_ERROR,
   "User profile mode: first target position in user profile is too far from current position"},
  {PI_EXT_PROFILE_ACTIVE, "Controller is (already) in user profile mode"},
  {PI_EXT_PROFILE_INDEX_OUT_OF_RANGE,
   "User profile mode: block or data set index out of allowed range"},
  {PI_PROFILE_GENERATOR_NO_PROFILE, "ProfileGenerator: No profile has been created yet"},
  {PI_PROFILE_GENERATOR_OUT_OF_LIMITS,
   "ProfileGenerator: Generated profile exceeds limits of one or both axes"},
  {PI_PROFILE_GENERATOR_UNKNOWN_PARAMETER,
   "ProfileGenerator: Unknown parameter ID in Set/Get Parameter command"},
  {PI_PROFILE_GENERATOR_PAR_OUT_OF_RANGE, "ProfileGenerator: Parameter out of allowed range"},
  {PI_EXT_PROFILE_OUT_OF_MEMORY, "User profile mode: out of memory"},
  {PI_EXT_PROFILE_WRONG_CLUSTER, "User profile mode: cluster is not assigned to this axis"},
  {PI_UNKNOWN_CLUSTER_IDENTIFIER, "Unknown cluster identifier"},
  {PI_INVALID_DEVICE_DRIVER_VERSION,
   "The installed device driver doesn't match the required version. Please see the documentation "
   "to determine the required device driver version."},
  {PI_INVALID_LIBRARY_VERSION,
   "The library used doesn't match the required version. Please see the documentation to determine "
   "the required library version."},
  {PI_INTERFACE_LOCKED,
   "The interface is currently locked by another function. Please try again later."},
  {PI_PARAM_DAT_FILE_INVALID_VERSION,
   "Version of parameter DAT file does not match the required version. Current files are available "
   "at www.pi.ws."},
  {PI_CANNOT_WRITE_TO_PARAM_DAT_FILE,
   "Cannot write to parameter DAT file to store user defined stage type."},
  {PI_CANNOT_CREATE_PARAM_DAT_FILE,
   "Cannot create parameter DAT file to store user defined stage type."},
  {PI_PARAM_DAT_FILE_INVALID_REVISION, "Parameter DAT file does not have correct revision."},
  {PI_USERSTAGES_DAT_FILE_INVALID_REVISION, "User stages DAT file does not have correct revision."},
  {PI_SOFTWARE_TIMEOUT,
   "Timeout Error. Some lengthy operation did not finish within expected time."},
  {PI_WRONG_DATA_TYPE, "A function argument has an unexpected data type."},
  {PI_DIFFERENT_ARRAY_SIZES, "Length of data arrays is different."},
  {PI_PARAM_NOT_FOUND_IN_PARAM_DAT_FILE, "Parameter value not found in parameter DAT file."},
  {PI_MACRO_RECORDING_NOT_ALLOWED_IN_THIS_MODE,
   "Macro recording is not allowed in this mode of operation."},
  {PI_USER_CANCELLED_COMMAND, "Command cancelled by user input."},
  {PI_TOO_FEW_GCS_DATA, "Controller sent too few GCS data sets"},
  {PI_TOO_MANY_GCS_DATA, "Controller sent too many GCS data sets"},
  {PI_GCS_DATA_READ_ERROR, "Communication error while reading GCS data"},
  {PI_WRONG_NUMBER_OF_INPUT_ARGUMENTS, "Wrong number of input arguments."},
  {PI_FAILED_TO_CHANGE_CCL_LEVEL, "Change of command level has failed."},
  {PI_FAILED_TO_SWITCH_OFF_SERVO, "Switching off the servo mode has failed."},
  {PI_FAILED_TO_SET_SINGLE_PARAMETER_WHILE_PERFORMING_CST,
   "A parameter could not be set while performing CST: CST was not performed (parameters remain "
   "unchanged)."},
  {PI_ERROR_CONTROLLER_REBOOT, "Connection could not be reestablished after reboot."},
  {PI_ERROR_AT_QHPA, "Sending HPA? or receiving the response has failed."},
  {PI_QHPA_NONCOMPLIANT_WITH_GCS, "HPA? response does not comply with GCS2 syntax."},
  {PI_FAILED_TO_READ_QSPA,
   "Response to SPA? could not be received.Response to SPA? could not be received."},
  {PI_PAM_FILE_WRONG_VERSION, "Version of PAM file cannot be handled (too old or too new)"},
  {PI_PAM_FILE_INVALID_FORMAT, "PAM file does not contain required data in PAM-file format"},
  {PI_INCOMPLETE_INFORMATION, "Information does not contain all required data"},
  {PI_NO_VALUE_AVAILABLE, "No value for parameter available"},
  {PI_NO_PAM_FILE_OPEN, "No PAM file is open"},
  {PI_INVALID_VALUE, "Invalid value"},
  {PI_UNKNOWN_PARAMETER, "Unknown parameter"},
  {PI_RESPONSE_TO_QSEP_FAILED, "Response to SEP? could not be received."},
  {PI_RESPONSE_TO_QSPA_FAILED,
   "Response to SPA? could not be received.Response to SPA? could not be received."},
  {PI_ERROR_IN_CST_VALIDATION,
   "Error while performing CST: One or more parameters were not set correctly."},
  {PI_ERROR_PAM_FILE_HAS_DUPLICATE_ENTRY_WITH_DIFFERENT_VALUES,
   "PAM file has duplicate entry with different values."},
  {PI_ERROR_FILE_NO_SIGNATURE, "File has no signature"},
  {PI_ERROR_FILE_INVALID_SIGNATURE, "File has invalid signature"},
  {PI_PARAMETER_DB_INVALID_STAGE_TYPE_FORMAT,
   "PI stage database: String containing stage type and description has invalid format."},
  {PI_PARAMETER_DB_SYSTEM_NOT_AVAILABLE,
   "PI stage database: Database does not contain the selected stage type for the connected "
   "controller."},
  {PI_PARAMETER_DB_FAILED_TO_ESTABLISH_CONNECTION,
   "PI stage database: Establishing the connection has failed."},
  {PI_PARAMETER_DB_COMMUNICATION_ERROR,
   "PI stage database: Communication was interrupted (e.g. because database was deleted)."},
  {PI_PARAMETER_DB_ERROR_WHILE_QUERYING_PARAMETERS, "PI stage database: Querying data failed."},
  {PI_PARAMETER_DB_SYSTEM_ALREADY_EXISTS,
   "PI stage database: System already exists. Rename stage and try again."},
  {PI_PARAMETER_DB_QHPA_CONTANS_UNKNOWN_PAM_IDS,
   "PI stage database: Response to HPA? contains unknown parameter IDs."},
  {PI_PARAMETER_DB_AND_QHPA_ARE_INCONSISTENT,
   "PI stage database: Inconsistency between database and response to HPA?."},
  {PI_PARAMETER_DB_SYSTEM_COULD_NOT_BE_ADDED, "PI stage database: Stage has not been added."},
  {PI_PARAMETER_DB_SYSTEM_COULD_NOT_BE_REMOVED, "PI stage database: Stage has not been removed."},
  {PI_PARAMETER_DB_CONTROLLER_DB_PARAMETERS_MISMATCH,
   "Controller does not support all stage parameters stored in PI stage database. No parameters "
   "were set."},
  {PI_PARAMETER_DB_DATABASE_IS_OUTDATED,
   "The version of PISTAGES3.DB stage database is out of date. Please update via PIUpdateFinder. "
   "No parameters were set."},
  {PI_PARAMETER_DB_AND_HPA_MISMATCH_STRICT,
   "Mismatch between number of parameters present in stage database and available in controller "
   "interface. No parameters were set."},
  {PI_PARAMETER_DB_AND_HPA_MISMATCH_LOOSE,
   "Mismatch between number of parameters present in stage database and available in controller "
   "interface. Some parameters were ignored."},
  {PI_PARAMETER_DB_FAILED_TO_SET_PARAMETERS_CORRECTLY,
   "One or more parameters could not be set correctly on the controller."},
  {PI_PARAMETER_DB_MISSING_PARAMETER_DEFINITIONS_IN_DATABASE,
   "One or more parameter definitions are not present in stage database. Please update "
   "PISTAGES3.DB via PIUpdateFinder. Missing parameters were ignored."},
};
} // namespace pi_hexapod_control
