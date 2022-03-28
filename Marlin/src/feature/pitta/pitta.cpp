/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfig.h"

#if HAS_PITTA_MMU

#include "pitta.h"



#include "../../gcode/gcode.h"
#include "../../lcd/marlinui.h"
#include "../../libs/buzzer.h"
#include "../../libs/nozzle.h"
#include "../../module/temperature.h"
#include "../../module/planner.h"
#include "../../module/stepper/indirection.h"
#include "../../MarlinCore.h"

PITTA pitta;
#define MODEL_E3V2_PRO32  // e3v2 and e3 pro 32bit main board. 
#define DEV_NCMD_PRINT
#undef DEV_NCMD_PRINT
// #define MODEL_E3PRO  // too small memory, so stopped to port for e3 pro uses 8 bit main board. 






#define PITTA_CMD_TIMEOUT 45000UL // Timeout for command
#define PITTA_FS_TIMEOUT 5000UL   // Timeout for filament sensor command

#define PITTA_CMD_NONE 0
#define PITTA_CMD_T0 0x10 // Tool Index
#define PITTA_CMD_T1 0x11 // Tool Index
#define PITTA_CMD_T2 0x12 // Tool Index
#define PITTA_CMD_T3 0x13 // Tool Index
#define PITTA_CMD_T4 0x14 // Tool Index
#define PITTA_CMD_T5 0x15 // Tool Index
#define PITTA_CMD_T6 0x16 // Tool Index
#define PITTA_CMD_T7 0x17 // Tool Index
#define PITTA_CMD_L0 0x20 // Load Filament
#define PITTA_CMD_L1 0x21 // Load Filament
#define PITTA_CMD_L2 0x22 // Load Filament
#define PITTA_CMD_L3 0x23 // Load Filament
#define PITTA_CMD_L4 0x24 // Load Filament
#define PITTA_CMD_L5 0x25 // Load Filament
#define PITTA_CMD_L6 0x26 // Load Filament
#define PITTA_CMD_L7 0x27 // Load Filament
#define PITTA_CMD_C 0x30  // Change Tool
#define PITTA_CMD_R 0x40  // Retract Filament
#define PITTA_CMD_E 0x50  // Extrude Filament

#define PITTA_SND_V 0x60 // Read Version
#define PITTA_SND_F 0x70 // Read Filament Sensor

#define PITTA_NO_TOOL 99

// #define mExtruder_dir(dir) WRITE(E0_DIR_PIN, dir)
#define mExtruder_step(state) E0_STEP_WRITE(state)

#define EXT_SLOW_LOAD 1800//1800 //1400
#define ABS_MOT_TRIGGER_CNT_DUTY 20000*CLK_MOD//300*CLK_MOD   //500 //30000
#define ABS_COMM_MASTER_CNT_DUTY 15000*CLK_MOD  //30000
#define ABS_COMM_RECEIVE_CNT_DUTY 3500*CLK_MOD  //5000 //10000
#define ABS_COMM_SEND_CNT_DUTY 5000*CLK_MOD     //10000
#define MATERIAL_OFFSET_BASE 38000      //82500










#ifdef MODEL_E3PRO

#define SPREAD_DIR LOW
#define SPREAD_RES_DIR HIGH
#define NOM_DIR LOW
#define INV_DIR HIGH
#define ONE_W_CMD_PIN Y_MIN_PIN //X_MIN_PIN //Y_MIN_PIN



#define MUL_V 1 //8
#define WAIT_MUL_V 1 //4
#define CLK_MOD 1
#define REF_CLK_BASE 5000


#define PITTA_MAIN_TICK TCNT1




typedef int16_t celsius_t;

void pitta_planner_sync() {
  stepper.synchronize();
}

void pitta_wtcdog_reset() {
  watchdog_reset();
  // watchdog_refresh();
}

bool is_possible_to_set_output() {
  pinMode(ONE_W_CMD_PIN, INPUT_PULLUP);
  delay(1);
  return READ(ONE_W_CMD_PIN);
}

bool confirmed_set_output_low() {
  pinMode(ONE_W_CMD_PIN, INPUT_PULLUP);
  delay(1);
  bool b_flag = false;
  int expire_cnt = 0;
  for (int i = 0; i<100; i++) {
    b_flag = READ(ONE_W_CMD_PIN);
    pitta_wtcdog_reset();
    if (!b_flag) {
      i = 0;
    }
    delay(1);
    expire_cnt++;
    if (expire_cnt>10000) {
      SERIAL_ECHOLN("output set expire, force output setting.. dangerous");
      pinMode(ONE_W_CMD_PIN, OUTPUT);
      WRITE(ONE_W_CMD_PIN, LOW);
      return false;
    }
  }
  SET_OUTPUT(ONE_W_CMD_PIN);
  WRITE(ONE_W_CMD_PIN, LOW);
  return true;
}

bool confirmed_set_output_high() {
  pinMode(ONE_W_CMD_PIN, INPUT_PULLUP);
  delay(1);
  bool b_flag = false;
  int expire_cnt = 0;
  for (int i = 0; i<200; i++) {
    b_flag = READ(ONE_W_CMD_PIN);
    pitta_wtcdog_reset();
    if (!b_flag) {
      i = 0;
    }
    delay(1);
    expire_cnt++;
    if (expire_cnt>10000) {
      SERIAL_ECHOLN("output set expire, force output setting.. dangerous");
      pinMode(ONE_W_CMD_PIN, OUTPUT);
      WRITE(ONE_W_CMD_PIN, LOW);
      return false;
    }
  }
  SET_OUTPUT(ONE_W_CMD_PIN);
  WRITE(ONE_W_CMD_PIN, HIGH);
  return true;
}

void wait_for_out_set_possible(){
    while (!is_possible_to_set_output()) {
    delay(50);
    pitta_wtcdog_reset();
    SERIAL_ECHOLN("RETRY OUT_PORT SET");
  }    
}


void pitta_req_manage_heater_update() {
  thermalManager.manage_heater();
}

void pitta_ui_thermal_update() {
  lcd_update();
  thermalManager.manage_heater();
}



void pitta_ui_update() {
  lcd_update();
}

int16_t pitta_get_temp(int ext_num) {
  return Temperature::target_temperature[ext_num];
}

int16_t pitta_get_celsius(int ext_num) {
  return thermalManager.current_temperature[ext_num];//
  // return Temperature::target_temperature[ext_num];
}


int16_t pitta_get_temp_bed () {
  return Temperature::target_temperature_bed;
}

void pitta_set_temp(int16_t temp, uint8_t ext_num) {
  thermalManager.setTargetHotend(temp, ext_num);
}

void pitta_set_temp_bed(int16_t temp) {
  thermalManager.setTargetBed(temp);
}

bool pitta_read_y_dir() {
  return READ(Y_DIR_PIN);
}

void pitta_enable_e0() {
  enable_E0();
}


#endif

#ifdef MODEL_E3V2_PRO32

#define SPREAD_DIR HIGH
#define SPREAD_RES_DIR LOW
#define NOM_DIR HIGH
#define INV_DIR LOW

#define MUL_V 1 //8
#define EXT_CM_M 1000//1020
#define EXT_MM_M 100//102
#define MAT_OFFSET -10
#define WAIT_MUL_V 1 //4
#define CLK_MOD 0.1//0.1 //2
#define REF_CLK_BASE 5000


#define PITTA_MAIN_TICK HAL_timer_get_count(MF_TIMER_PITTA) // 2000 per 1ms

HAL_PITTA_TIMER_ISR() {
  HAL_timer_isr_prologue(MF_TIMER_PITTA);
  HAL_timer_isr_epilogue(MF_TIMER_PITTA);
}

void pitta_planner_sync() {
  planner.synchronize();
}

void pitta_wtcdog_reset() {
  // watchdog_reset();
  watchdog_refresh();
}

bool is_possible_to_set_output() {
  SET_INPUT_PULLUP(ONE_W_CMD_PIN);
  delay(1);
  return READ(ONE_W_CMD_PIN);
}

bool confirmed_set_output_low() {
  SET_INPUT_PULLUP(ONE_W_CMD_PIN);
  delay(1);
  bool b_flag = false;
  int expire_cnt = 0;
  for (int i = 0; i<100; i++) {
    pitta_wtcdog_reset();
    b_flag = READ(ONE_W_CMD_PIN);
    if (!b_flag) {
      i = 0;
    }
    delay(1);
    expire_cnt++;
    if (expire_cnt>10000) {
      SERIAL_ECHOLN("output set expire, force output setting.. dangerous");
      pinMode(ONE_W_CMD_PIN, OUTPUT);
      WRITE(ONE_W_CMD_PIN, LOW);
      return false;
    }
  }
  SET_OUTPUT(ONE_W_CMD_PIN);
  WRITE(ONE_W_CMD_PIN, LOW);
  return true;
}

bool confirmed_set_output_high() {
  SET_INPUT_PULLUP(ONE_W_CMD_PIN);
  delay(1);
  bool b_flag = false;
  int expire_cnt = 0;
  for (int i = 0; i<200; i++) {
    b_flag = READ(ONE_W_CMD_PIN);
    pitta_wtcdog_reset();
    if (!b_flag) {
      i = 0;
    }
    delay(1);
    expire_cnt++;
    if (expire_cnt>10000) {
      SERIAL_ECHOLN("output set expire, force output setting.. dangerous");
      pinMode(ONE_W_CMD_PIN, OUTPUT);
      WRITE(ONE_W_CMD_PIN, LOW);
      return false;
    }
  }
  SET_OUTPUT(ONE_W_CMD_PIN);
  WRITE(ONE_W_CMD_PIN, HIGH);
  return true;
}

void wait_for_out_set_possible(){
    while (!is_possible_to_set_output()) {
    delay(50);
    pitta_wtcdog_reset();
    SERIAL_ECHOLN("RETRY OUT_PORT SET");
  }    
}


void pitta_ui_update() {
  // lcd_update();
  TERN(HAS_DWIN_E3V2_BASIC, DWIN_Update(), ui.update());
  // thermalManager.manage_heater();
}

// int heat_req_cnt = 0;
void pitta_req_manage_heater_update() {
  thermalManager.manage_heater();
  // heat_req_cnt++;
  // if (heat_req_cnt>1000) heat_req_cnt = 0;
  // SERIAL_ECHOLN(heat_req_cnt);
}

void pitta_ui_thermal_update() {
  // lcd_update();
  TERN(HAS_DWIN_E3V2_BASIC, DWIN_Update(), ui.update());
  thermalManager.manage_heater();
}

int16_t pitta_get_temp(int ext_num) {
  return thermalManager.temp_hotend[ext_num].target;
  // return Temperature::target_temperature[ext_num];
}

int16_t pitta_get_celsius(int ext_num) {
  return thermalManager.temp_hotend[ext_num].celsius;
  // return Temperature::target_temperature[ext_num];
}

int16_t pitta_get_temp_bed() {
  return thermalManager.temp_bed.target;
  // return Temperature::target_temperature_bed;
}

void pitta_set_temp(int16_t temp, uint8_t ext_num) {
  thermalManager.setTargetHotend(temp, ext_num);
}

void pitta_set_temp_bed(int16_t temp) {
  thermalManager.setTargetBed(temp);
}

bool pitta_read_y_dir() {
  return Y_DIR_READ();
  // return READ(Y_DIR_PIN);
}

void pitta_enable_e0() {
  ENABLE_AXIS_E0();
}

#endif

bool cur_ext_dir = false;
void mExtruder_dir(bool dir) {
  if (cur_ext_dir!=dir) {
    cur_ext_dir = dir;
    mExtruder_step(LOW);
    delay(5);
    E0_DIR_WRITE(INVERT_E0_DIR ? !dir : dir);
    delay(5);
  }
}




long int mot_ext_remain_step = 0, init_mot_ext_remain_step = 0, feed_fast_cnt = 0;
int ext_load_damp = EXT_SLOW_LOAD;
unsigned int i_ABS_MOT_TRIGGER_CNT_DUTY = ABS_MOT_TRIGGER_CNT_DUTY;
long int change_talk_expire_cnt = 0;
bool b_pitta_jammed = false;
bool b_jam_recovered = false;
bool b_last_jam_sensor_state = false;
celsius_t temp_temp_extruder = 0, temp_temp_bed = 0;
unsigned int diplay_update_cnt = 0;
long int jam_expire_cnt = 0;
int mat_det_hyst_cnt = 0;
extern bool b_change_done, b_selector_done;
long int extrude_init_dist = 0;
int resend_retry_cnt = 0;
long material_offset = 0;
long material_tune_offset = 0;
bool b_req_retract_fully = false;

// PITTA extrude base offset
int16_t pitta_extrude_offset = -1;
int16_t pitta_extrude_1st_retract_dist = 9;
int16_t pitta_extrude_lowest_extrude_dist = 10;
int16_t pitta_extrude_return_spd = 0;
int16_t pitta_retract_full_offset = 6;
int16_t pitta_extrude_turn_val = 0;//6;//10
int16_t pitta_extrude_turn_B_val = 2;//9;//10
int16_t pitta_extrude_turn_C_val = 0;//0;//10

// int16_t pitta_val_1 = 0, pitta_val_2 = 0, pitta_val_3 = 0, pitta_val_4 = 0, pitta_val_5 = 0, pitta_val_6 = 0,pitta_val_7 = 0;
int16_t tb_len = 0, resrv1 = 0, resrv2 = 0, resrv3 = 0, resrv4 = 0, ptrn_n = 0;


int ms_hyst_cnt = 0;
bool b_ms_val = false, b_ms_hyst_val = true;
bool chk_material() {
    b_ms_val = !READ(ONE_W_CMD_PIN);
    if (!b_ms_val) {
      ms_hyst_cnt++;
      if (ms_hyst_cnt>50) {
        ms_hyst_cnt = 0;
        b_ms_hyst_val = b_ms_val;
      }
    }
    else {
      ms_hyst_cnt = 0;
      b_ms_hyst_val = b_ms_val;
    }
    return b_ms_hyst_val;
}
bool b_abnormal_pitta_power = false;
bool io_chk_n_set_as_output (int pin) {
  int pwr_chk_cnt = 0;
    SET_INPUT_PULLDOWN(pin);
    delay(10);
    while (!READ(pin)) {
      delay(20);
      pwr_chk_cnt++;
      pitta_wtcdog_reset();
      pitta_req_manage_heater_update();
      if (pwr_chk_cnt>200) {
        // bad power. 
        // return false;
        b_abnormal_pitta_power = true;
        break;
      }
    }
    pinMode(pin, OUTPUT);
    delay(10);
    return true;
}

#define DAMP_TREMBLE 60*MUL_V  //80 //60//100
unsigned int temp_expipre_accum_cnt = 0;

void ext_flat(bool dir, unsigned int spd, long int last_cnt) {
  if (last_cnt<0) return;
  mExtruder_dir(dir);
  pitta_wtcdog_reset();
  last_cnt = (last_cnt >> 1)*MUL_V;
  while (last_cnt > 0) {
    last_cnt--;
    mExtruder_step(HIGH);
    delayMicroseconds(spd + 1);
    mExtruder_step(LOW);
    delayMicroseconds(spd);   

    temp_expipre_accum_cnt = temp_expipre_accum_cnt + (spd>>3) + 1;
    if (temp_expipre_accum_cnt>4000) {
      temp_expipre_accum_cnt = temp_expipre_accum_cnt - 4000;
      pitta_req_manage_heater_update();
    }       
  }  
} 

#define STAY_DUTY 60
void ext_ramp_acc(bool dir, unsigned int st_spd, unsigned int ed_spd, long int last_cnt) {
  int spd = 0, delta = 0, time_dist, add_cnt = 0;
  mExtruder_dir(dir);
  pitta_wtcdog_reset();
  if (last_cnt<0) return;
  last_cnt = (last_cnt >> 1)*MUL_V;
  spd = st_spd;

  while (last_cnt > 0) {
    if (spd>ed_spd) {
      spd = spd - spd/20;
    }
    else if (spd<ed_spd) {
      spd = spd + spd/20;
    }
    last_cnt--;
    mExtruder_step(HIGH);
    delayMicroseconds(spd + 1);
    mExtruder_step(LOW);
    delayMicroseconds(spd);   

    temp_expipre_accum_cnt = temp_expipre_accum_cnt + (spd>>3) + 1;
    if (temp_expipre_accum_cnt>4000) {
      temp_expipre_accum_cnt = temp_expipre_accum_cnt - 4000;
      pitta_req_manage_heater_update();
    }       
  }  
} 

void snap_ext_damp(bool dir, unsigned int spd, long int last_cnt)
{
  if (last_cnt<0) return;
  ext_flat(dir, spd, last_cnt);
  return;
  last_cnt = (last_cnt >> 1)*MUL_V;
  spd = spd/MUL_V;
  mExtruder_dir(dir);
  pitta_wtcdog_reset();
  unsigned int cnt = 0;
  unsigned int var_spd = 0;
  int wtcd_cnt = 0;
  int damp_tr = 0;
  


  if (last_cnt > 100*MUL_V) {
    cnt = DAMP_TREMBLE;
    var_spd = spd + DAMP_TREMBLE;
    damp_tr = cnt;
  }
  else {
    cnt = last_cnt >> 1;
    var_spd = spd + cnt; 
    damp_tr = cnt;  
  }

  while (cnt > 0)
  {
    wtcd_cnt++;
    if (wtcd_cnt > 500*MUL_V)
    {
      wtcd_cnt = 0;
      pitta_wtcdog_reset();
    }
    cnt--;
    mExtruder_step(HIGH);
    delayMicroseconds(var_spd + 1);
    mExtruder_step(LOW);
    delayMicroseconds(var_spd);
    temp_expipre_accum_cnt = temp_expipre_accum_cnt + (var_spd>>3);
    if (temp_expipre_accum_cnt>4000) {
      temp_expipre_accum_cnt = temp_expipre_accum_cnt - 4000;
      pitta_req_manage_heater_update();
    }
    if (var_spd > spd)
    {
      var_spd--;
    }
    last_cnt--;
  }
  while (last_cnt - damp_tr > 0)
  {
    wtcd_cnt++;
    if (wtcd_cnt > 500*MUL_V)
    {
      wtcd_cnt = 0;
      pitta_wtcdog_reset();
      pitta_req_manage_heater_update();
    }
    last_cnt--;
    mExtruder_step(HIGH);
    delayMicroseconds(spd + 1);
    mExtruder_step(LOW);
    delayMicroseconds(spd);
    temp_expipre_accum_cnt = temp_expipre_accum_cnt + (spd>>3);
    if (temp_expipre_accum_cnt>4000) {
      temp_expipre_accum_cnt = temp_expipre_accum_cnt - 4000;
      pitta_req_manage_heater_update();
    }
  }
  cnt = damp_tr;
  var_spd = spd;
  if (cnt < last_cnt) cnt = last_cnt;
  while (cnt > 0)
  {
    cnt--;
    mExtruder_step(HIGH);
    delayMicroseconds(var_spd + 1);
    mExtruder_step(LOW);
    delayMicroseconds(var_spd);
    if (var_spd < damp_tr)
    {
      var_spd++;
    }
    temp_expipre_accum_cnt = temp_expipre_accum_cnt + (var_spd>>3);
    if (temp_expipre_accum_cnt>4000) {
      temp_expipre_accum_cnt = temp_expipre_accum_cnt - 4000;
      pitta_req_manage_heater_update();
    }    
  }
  pitta_wtcdog_reset();
}

typedef enum COM_SR_CMD {CHG_DONE = 27, MSG_DONE, RCV_NONE = 32, RESEND_REQ, WAIT_REQ,JAMMED, RDY_REQ = 127, PTMSG_OK, DATA_W_NO_CMD, COM_CONTINUE, TUNE_VAL1, TUNE_VAL2, TUNE_VAL3, TUNE_VAL4, TUNE_VAL5, TUNE_VAL6, TUNE_VAL7, PARM_0_SET_REQ, PARM_1_SET_REQ } COM_SR_CMD;
typedef enum PROCEESS_CUR_STATE {PROC_CHANGE_DONE = 200, PROC_INIT_STARTED , PROC_WORKING, PROC_IDLE, PROC_MSG_OK, RETRY_PROC, PROC_CONTINUE } PROCEED_CUR_STATE;
COM_SR_CMD srcv_cmd;
PROCEESS_CUR_STATE proc_state;

bool PITTA::pitta_enabled, PITTA::pitta_ready, PITTA::pitta_print_saved, PITTA::pitta_sel_req = false, PITTA::b_pitta_loop_lock = false, PITTA::b_e0_dir, PITTA::b_init_pitta = false, PITTA::b_init_pitta_initialized = false, PITTA::b_material_detected_from_pitta = false, PITTA::b_pitta_lock = false, PITTA::b_slave_as_sensor = false, PITTA::b_slave_as_invalid_sensor = false, PITTA::pitta_stop_req, PITTA::b_retract_ready, PITTA::b_pitta_just_enabled;

int /* PITTA::send_M_byte_remain_num = 0, */ PITTA::e0_spd = 0, PITTA::change_turn_val = 0;

unsigned int PITTA::prev_main_sys_tick = 0, PITTA::main_sys_tick = 0;
unsigned int PITTA::loop_diff_tick = 0;
unsigned int PITTA::abs_comm_receive_remain_tick = 0;
unsigned int PITTA::abs_comm_send_remain_tick = 0;
unsigned int PITTA::abs_issue_trigger_remain_tick = 0;
int PITTA::must_ready_hear_cnt = 0;
bool PITTA::b_ok_ready_to_say = false;
bool PITTA::b_on_hearing = false, PITTA::b_on_saying = false;
long PITTA::receive_bit_shift = 0;
long PITTA::receive_byte = 0;
bool PITTA::b_send_bit = false;
int PITTA::send_hi_turn = 0;
int PITTA::send_lo_turn = 0;
long PITTA::send_packet = 0, PITTA::sent_packet = 0;
unsigned int PITTA::send_cmd = 0, PITTA::send_data = 0, PITTA::send_chk = 0;    
long PITTA::send_bit_shift = 0;
bool PITTA::b_start_send_bit = false;
unsigned int PITTA::receive_tick_hi_cnt = 0;
unsigned int PITTA::receive_tick_lo_cnt = 0;
long PITTA::received_packet = 0;
unsigned int PITTA::received_cmd;
int PITTA::received_data;
bool PITTA::b_byte_receive_done = false;

int PITTA::pitta_log_cnt = 0;

uint8_t PITTA::cmd, PITTA::last_cmd, PITTA::extruder;
uint8_t PITTA::pitta_state = 0;
volatile int8_t PITTA::fila_sens = 1;
volatile bool PITTA::fila_runout_valid;
int16_t PITTA::version = -1;
millis_t PITTA::prev_request, PITTA::prev_FS_request;
char PITTA::rx_buffer[PITTA_RX_SIZE], PITTA::tx_buffer[PITTA_TX_SIZE];
bool b_snap_done = false;
bool b_material_out_tried = false;
extern int16_t pitta_retract_full_offset;
extern int16_t pitta_extrude_turn_val, pitta_extrude_turn_B_val,pitta_extrude_turn_C_val, pitta_extrude_1st_retract_dist, pitta_extrude_lowest_extrude_dist, pitta_extrude_return_spd;



#define SPREAD_WIDTH 250*MUL_V
#ifdef DEV_NCMD_PRINT
  int wrong_packet_cnt = 0, wrong_cmd_cnt = 0;
#endif
bool b_y_dir = false;
bool b_y_spread = false;
int y_shift_pos = 0;
bool b_use_spread_tower = false;//true;
bool b_stop_active = false;
void ext_snap()
{
  b_snap_done = false;
  pitta_ui_thermal_update();
  temp_temp_extruder = pitta_get_temp(0);
  
  pitta_ui_thermal_update();
 
  if (b_use_spread_tower) {    
    Y_STEP_WRITE(LOW);
    delayMicroseconds(200);
    b_y_dir = pitta_read_y_dir();//Y_DIR_READ;
    Y_DIR_WRITE(INVERT_Y_DIR ? !SPREAD_DIR : SPREAD_DIR);
    delayMicroseconds(200);    
    const int dec_lim = 200;
    int front_dec = dec_lim, rear_dec = 0;
    b_y_spread = false;
    y_shift_pos+= SPREAD_WIDTH;
    if (y_shift_pos>SPREAD_WIDTH*8) y_shift_pos = 0;
    int cur_pos = 0;
    for (cur_pos = 0;cur_pos<y_shift_pos;cur_pos++) {
      if (cur_pos<(y_shift_pos>>1)) {
        front_dec--;
        if (front_dec<0) front_dec = 0;
      }
      if (cur_pos>dec_lim||cur_pos>(y_shift_pos>>1)) {
        rear_dec++;
        if (rear_dec>dec_lim) rear_dec = dec_lim;
      }
      Y_STEP_WRITE(HIGH);
      delayMicroseconds(100+front_dec+rear_dec);
      Y_STEP_WRITE(LOW);
      delayMicroseconds(100+front_dec+rear_dec);
      pitta_ui_thermal_update();
    }
    Y_DIR_WRITE(INVERT_Y_DIR ? LOW : HIGH);
  }


#define TEST_PATTERN
#define FAST_PASS 65 //60
#define SLOW_PASS 170 //150 
#define TURN_PASS 270 // 250
#define COOL_ADD 500
#define DIP_MELT 200
  cur_ext_dir = INV_DIR;
  E0_DIR_WRITE(INVERT_E0_DIR ? !INV_DIR : INV_DIR);
  delay(2);
  int add = 0, add_cnt = 0, sub = 0;
  int add_bit = 0, sub_bit = 0;
  int dip_melt = 0;
  int l_turn = 0;

  
  pitta_req_manage_heater_update();  
  switch (ptrn_n) {
    case 0:
    {
      resrv2 = 45;
      ext_flat(INV_DIR, 100, 50);
      ext_flat(INV_DIR, 1000, 250);
      // l_turn = resrv2;//
      for (int i = 0; i<10; i++ ) { ////20//resrv2
        ext_flat(INV_DIR, TURN_PASS, 15);
        ext_flat(INV_DIR, SLOW_PASS, 10);
        ext_flat(INV_DIR, 110, 20);
        ext_flat(INV_DIR, 90, 20);
        ext_flat(INV_DIR, 80, 20);
        ext_flat(INV_DIR, 70, 15);
        ext_flat(INV_DIR, FAST_PASS, 10);
        ext_flat(INV_DIR, 70, 15);
        ext_flat(INV_DIR, 80, 20);
        ext_flat(INV_DIR, 90, 20);
        ext_flat(INV_DIR, 110, 20);
        ext_flat(INV_DIR, SLOW_PASS, 15);
        ext_flat(INV_DIR, TURN_PASS, 10);

        ext_flat(NOM_DIR, TURN_PASS, 10);
        ext_flat(NOM_DIR, SLOW_PASS, 15);
        ext_flat(NOM_DIR, 110, 20);
        ext_flat(NOM_DIR, 90, 20);
        ext_flat(NOM_DIR, 80, 20);
        ext_flat(NOM_DIR, 70, 15);
        ext_flat(NOM_DIR, FAST_PASS, 0);
        ext_flat(NOM_DIR, 70, 15);
        ext_flat(NOM_DIR, 80, 20);
        ext_flat(NOM_DIR, 90, 20);
        ext_flat(NOM_DIR, 110, 20);
        ext_flat(NOM_DIR, SLOW_PASS, 10);
        ext_flat(NOM_DIR, TURN_PASS, 15);
      }
      l_turn = resrv2;//
      for (int i = 0; i<l_turn; i++ ) { ////20//resrv2
        ext_flat(INV_DIR, TURN_PASS, 15);
        ext_flat(INV_DIR, SLOW_PASS, 10);
        ext_flat(INV_DIR, 110, 20);
        ext_flat(INV_DIR, 90, 20);
        ext_flat(INV_DIR, 80, 20);
        ext_flat(INV_DIR, 70, 15);
        if (i == 10) {
          ext_flat(INV_DIR, FAST_PASS, 170 + 7500);
        }
        else if ( i == 17) {
          ext_flat(INV_DIR, FAST_PASS, 170 + 500);
        }
        else if ( i == 28) {
          ext_flat(INV_DIR, FAST_PASS, 170 + 500);
        }
        else if ( i == 35) {
          ext_flat(INV_DIR, FAST_PASS, 170 + 500);
        }
        else {
          ext_flat(INV_DIR, FAST_PASS, 170);
        }
        ext_flat(INV_DIR, 70, 15);
        ext_flat(INV_DIR, 80, 20);
        ext_flat(INV_DIR, 90, 20);
        ext_flat(INV_DIR, 110, 20);
        ext_flat(INV_DIR, SLOW_PASS, 15);
        ext_flat(INV_DIR, TURN_PASS, 10);

        ext_flat(NOM_DIR, TURN_PASS, 10);
        ext_flat(NOM_DIR, SLOW_PASS, 15);
        ext_flat(NOM_DIR, 110, 20);
        ext_flat(NOM_DIR, 90, 20);
        ext_flat(NOM_DIR, 80, 20);
        ext_flat(NOM_DIR, 70, 15);
        ext_flat(NOM_DIR, FAST_PASS, 170);
        ext_flat(NOM_DIR, 70, 15);
        ext_flat(NOM_DIR, 80, 20);
        ext_flat(NOM_DIR, 90, 20);
        ext_flat(NOM_DIR, 110, 20);
        ext_flat(NOM_DIR, SLOW_PASS, 10);
        ext_flat(NOM_DIR, TURN_PASS, 15);
      } 
      ext_flat(NOM_DIR, 200,8700);//8900
      ext_flat(NOM_DIR, 300,300);
      ext_flat(NOM_DIR, 100*27,100);//resrv3 23
 
      ext_flat(INV_DIR, 300, 12);
      ext_flat(INV_DIR, 250, 12);
      ext_flat(INV_DIR, 200, 10);
      ext_flat(INV_DIR, 170, 10);
      ext_flat(INV_DIR, 120, 10);
      ext_flat(INV_DIR, 100, 15);
      ext_flat(INV_DIR, 80, 20);
      ext_flat(INV_DIR, 70, 30);
      ext_flat(INV_DIR, 60, 50);
      ext_flat(INV_DIR, 50, 70);
      ext_flat(INV_DIR, 45, 60);
      ext_flat(INV_DIR, 45, 39000);

      pitta_set_temp((temp_temp_extruder), 0);
      ext_flat(INV_DIR, 45/* +pitta_extrude_return_spd */, 20000);//
      ext_flat(INV_DIR, 60/* +pitta_extrude_return_spd */, 10000);//

    }
    break;
    case 4:
    {
      // resrv2 = 45;
      ext_flat(INV_DIR, 100, 50);
      ext_flat(INV_DIR, 1000, 250);
      // l_turn = resrv2;//
      for (int i = 0; i<10; i++ ) { ////20//resrv2
        ext_flat(INV_DIR, TURN_PASS, 15);
        ext_flat(INV_DIR, SLOW_PASS, 10);
        ext_flat(INV_DIR, 110, 20);
        ext_flat(INV_DIR, 90, 20);
        ext_flat(INV_DIR, 80, 20);
        ext_flat(INV_DIR, 70, 15);
        ext_flat(INV_DIR, FAST_PASS, 10);
        ext_flat(INV_DIR, 70, 15);
        ext_flat(INV_DIR, 80, 20);
        ext_flat(INV_DIR, 90, 20);
        ext_flat(INV_DIR, 110, 20);
        ext_flat(INV_DIR, SLOW_PASS, 15);
        ext_flat(INV_DIR, TURN_PASS, 10);

        ext_flat(NOM_DIR, TURN_PASS, 10);
        ext_flat(NOM_DIR, SLOW_PASS, 15);
        ext_flat(NOM_DIR, 110, 20);
        ext_flat(NOM_DIR, 90, 20);
        ext_flat(NOM_DIR, 80, 20);
        ext_flat(NOM_DIR, 70, 15);
        ext_flat(NOM_DIR, FAST_PASS, 0);
        ext_flat(NOM_DIR, 70, 15);
        ext_flat(NOM_DIR, 80, 20);
        ext_flat(NOM_DIR, 90, 20);
        ext_flat(NOM_DIR, 110, 20);
        ext_flat(NOM_DIR, SLOW_PASS, 10);
        ext_flat(NOM_DIR, TURN_PASS, 15);
      }
      l_turn = resrv2;//
      for (int i = 0; i<l_turn; i++ ) { ////20//resrv2
        ext_flat(INV_DIR, TURN_PASS, 15);
        ext_flat(INV_DIR, SLOW_PASS, 10);
        ext_flat(INV_DIR, 110, 20);
        ext_flat(INV_DIR, 90, 20);
        ext_flat(INV_DIR, 80, 20);
        ext_flat(INV_DIR, 70, 15);
        if (i == 10) {
          ext_flat(INV_DIR, FAST_PASS, 170 + 7500);
        }
        else if ( i == 17) {
          ext_flat(INV_DIR, FAST_PASS, 170 + 500);
        }
        else if ( i == 28) {
          ext_flat(INV_DIR, FAST_PASS, 170 + 500);
        }
        else if ( i == 35) {
          ext_flat(INV_DIR, FAST_PASS, 170 + 500);
        }
        else {
          ext_flat(INV_DIR, FAST_PASS, 170);
        }
        ext_flat(INV_DIR, 70, 15);
        ext_flat(INV_DIR, 80, 20);
        ext_flat(INV_DIR, 90, 20);
        ext_flat(INV_DIR, 110, 20);
        ext_flat(INV_DIR, SLOW_PASS, 15);
        ext_flat(INV_DIR, TURN_PASS, 10);

        ext_flat(NOM_DIR, TURN_PASS, 10);
        ext_flat(NOM_DIR, SLOW_PASS, 15);
        ext_flat(NOM_DIR, 110, 20);
        ext_flat(NOM_DIR, 90, 20);
        ext_flat(NOM_DIR, 80, 20);
        ext_flat(NOM_DIR, 70, 15);
        ext_flat(NOM_DIR, FAST_PASS, 170);
        ext_flat(NOM_DIR, 70, 15);
        ext_flat(NOM_DIR, 80, 20);
        ext_flat(NOM_DIR, 90, 20);
        ext_flat(NOM_DIR, 110, 20);
        ext_flat(NOM_DIR, SLOW_PASS, 10);
        ext_flat(NOM_DIR, TURN_PASS, 15);
      }
      ext_flat(NOM_DIR, 200,8700);//8900
      ext_flat(NOM_DIR, 300,300);
      ext_flat(NOM_DIR, 100*resrv3,100);//resrv3 27

      ext_flat(INV_DIR, 300, 12);
      ext_flat(INV_DIR, 250, 12);
      ext_flat(INV_DIR, 200, 10);
      ext_flat(INV_DIR, 170, 10);
      ext_flat(INV_DIR, 120, 10);
      ext_flat(INV_DIR, 100, 15);
      ext_flat(INV_DIR, 80, 20);
      ext_flat(INV_DIR, 70, 30);
      ext_flat(INV_DIR, 60, 50);
      ext_flat(INV_DIR, 50, 70);
      ext_flat(INV_DIR, 45, 60);
      ext_flat(INV_DIR, 45, 39000);

      pitta_set_temp((temp_temp_extruder), 0);   
      ext_flat(INV_DIR, 45/* +pitta_extrude_return_spd */, 20000);//
      ext_flat(INV_DIR, 60/* +pitta_extrude_return_spd */, 10000);//  

    }
    break;
    case 100:
    {
      resrv2 = 45;
      ext_flat(INV_DIR, 100, 50);
      ext_flat(INV_DIR, 1000, 250);
      l_turn = resrv2;//
      for (int i = 0; i<10; i++ ) { ////20//resrv2
        ext_flat(INV_DIR, TURN_PASS, 15);
        ext_flat(INV_DIR, SLOW_PASS, 10);
        ext_flat(INV_DIR, 110, 20);
        ext_flat(INV_DIR, 90, 20);
        ext_flat(INV_DIR, 80, 20);
        ext_flat(INV_DIR, 70, 15);
        ext_flat(INV_DIR, FAST_PASS, 10);
        ext_flat(INV_DIR, 70, 15);
        ext_flat(INV_DIR, 80, 20);
        ext_flat(INV_DIR, 90, 20);
        ext_flat(INV_DIR, 110, 20);
        ext_flat(INV_DIR, SLOW_PASS, 15);
        ext_flat(INV_DIR, TURN_PASS, 10);

        ext_flat(NOM_DIR, TURN_PASS, 10);
        ext_flat(NOM_DIR, SLOW_PASS, 15);
        ext_flat(NOM_DIR, 110, 20);
        ext_flat(NOM_DIR, 90, 20);
        ext_flat(NOM_DIR, 80, 20);
        ext_flat(NOM_DIR, 70, 15);
        ext_flat(NOM_DIR, FAST_PASS, 0);
        ext_flat(NOM_DIR, 70, 15);
        ext_flat(NOM_DIR, 80, 20);
        ext_flat(NOM_DIR, 90, 20);
        ext_flat(NOM_DIR, 110, 20);
        ext_flat(NOM_DIR, SLOW_PASS, 10);
        ext_flat(NOM_DIR, TURN_PASS, 15);
      }
      l_turn = resrv2;//
      for (int i = 0; i<l_turn; i++ ) { ////20//resrv2
        ext_flat(INV_DIR, TURN_PASS, 15);
        ext_flat(INV_DIR, SLOW_PASS, 10);
        ext_flat(INV_DIR, 110, 20);
        ext_flat(INV_DIR, 90, 20);
        ext_flat(INV_DIR, 80, 20);
        ext_flat(INV_DIR, 70, 15);
        if (i == 10) {
          ext_flat(INV_DIR, FAST_PASS, 170 + 7500);
        }
        else if ( i == 17) {
          ext_flat(INV_DIR, FAST_PASS, 170 + 500);
        }
        else if ( i == 28) {
          ext_flat(INV_DIR, FAST_PASS, 170 + 500);
        }
        else if ( i == 35) {
          ext_flat(INV_DIR, FAST_PASS, 170 + 500);
        }
        else {
          ext_flat(INV_DIR, FAST_PASS, 170);
        }
        ext_flat(INV_DIR, 70, 15);
        ext_flat(INV_DIR, 80, 20);
        ext_flat(INV_DIR, 90, 20);
        ext_flat(INV_DIR, 110, 20);
        ext_flat(INV_DIR, SLOW_PASS, 15);
        ext_flat(INV_DIR, TURN_PASS, 10);

        ext_flat(NOM_DIR, TURN_PASS, 10);
        ext_flat(NOM_DIR, SLOW_PASS, 15);
        ext_flat(NOM_DIR, 110, 20);
        ext_flat(NOM_DIR, 90, 20);
        ext_flat(NOM_DIR, 80, 20);
        ext_flat(NOM_DIR, 70, 15);
        ext_flat(NOM_DIR, FAST_PASS, 170);
        ext_flat(NOM_DIR, 70, 15);
        ext_flat(NOM_DIR, 80, 20);
        ext_flat(NOM_DIR, 90, 20);
        ext_flat(NOM_DIR, 110, 20);
        ext_flat(NOM_DIR, SLOW_PASS, 10);
        ext_flat(NOM_DIR, TURN_PASS, 15);
      } 
      ext_flat(NOM_DIR, 200,8700);//8900
      ext_flat(NOM_DIR, 300,300);
      ext_flat(NOM_DIR, 100*27,100);//resrv3 23
 
      ext_flat(INV_DIR, 300, 12);
      ext_flat(INV_DIR, 250, 12);
      ext_flat(INV_DIR, 200, 10);
      ext_flat(INV_DIR, 170, 10);
      ext_flat(INV_DIR, 120, 10);
      ext_flat(INV_DIR, 100, 15);
      ext_flat(INV_DIR, 80, 20);
      ext_flat(INV_DIR, 70, 30);
      ext_flat(INV_DIR, 60, 50);
      ext_flat(INV_DIR, 50, 70);
      ext_flat(INV_DIR, 45, 60);
      ext_flat(INV_DIR, 45, 10000);

      l_turn = 20;
      for (int i = 0; i<l_turn; i++ ) { ////20//resrv2
        ext_flat(INV_DIR, TURN_PASS, 15);
        ext_flat(INV_DIR, SLOW_PASS, 10);
        ext_flat(INV_DIR, 110, 20);
        ext_flat(INV_DIR, 90, 20);
        ext_flat(INV_DIR, 80, 20);
        ext_flat(INV_DIR, 70, 15);
        ext_flat(INV_DIR, FAST_PASS, 200);
        ext_flat(INV_DIR, 70, 15);
        ext_flat(INV_DIR, 80, 20);
        ext_flat(INV_DIR, 90, 20);
        ext_flat(INV_DIR, 110, 20);
        ext_flat(INV_DIR, SLOW_PASS, 15);
        ext_flat(INV_DIR, TURN_PASS, 10);

        ext_flat(NOM_DIR, TURN_PASS, 10);
        ext_flat(NOM_DIR, SLOW_PASS, 15);
        ext_flat(NOM_DIR, 110, 20);
        ext_flat(NOM_DIR, 90, 20);
        ext_flat(NOM_DIR, 80, 20);
        ext_flat(NOM_DIR, 70, 15);
        ext_flat(NOM_DIR, FAST_PASS, 200);
        ext_flat(NOM_DIR, 70, 15);
        ext_flat(NOM_DIR, 80, 20);
        ext_flat(NOM_DIR, 90, 20);
        ext_flat(NOM_DIR, 110, 20);
        ext_flat(NOM_DIR, SLOW_PASS, 10);
        ext_flat(NOM_DIR, TURN_PASS, 15);
      }
      ext_flat(INV_DIR, 300, 12);
      ext_flat(INV_DIR, 250, 12);
      ext_flat(INV_DIR, 200, 10);
      ext_flat(INV_DIR, 170, 10);
      ext_flat(INV_DIR, 120, 10);
      ext_flat(INV_DIR, 100, 15);
      ext_flat(INV_DIR, 80, 20);
      ext_flat(INV_DIR, 70, 30);
      ext_flat(INV_DIR, 60, 50);
      ext_flat(INV_DIR, 50, 70);
      ext_flat(INV_DIR, 45, 60);
      ext_flat(INV_DIR, 45, 10000);     //29000

      pitta_set_temp((temp_temp_extruder), 0);   
      ext_flat(INV_DIR, 45/* +pitta_extrude_return_spd */, 20000);//
      ext_flat(INV_DIR, 60/* +pitta_extrude_return_spd */, 10000);//  
 
    }
    break;
    default:
    {
      resrv2 = 45;
      ext_flat(INV_DIR, 100, 50);
      ext_flat(INV_DIR, 1000, 250);
      // l_turn = resrv2;//
      for (int i = 0; i<10; i++ ) { ////20//resrv2
        ext_flat(INV_DIR, TURN_PASS, 15);
        ext_flat(INV_DIR, SLOW_PASS, 10);
        ext_flat(INV_DIR, 110, 20);
        ext_flat(INV_DIR, 90, 20);
        ext_flat(INV_DIR, 80, 20);
        ext_flat(INV_DIR, 70, 15);
        ext_flat(INV_DIR, FAST_PASS, 10);
        ext_flat(INV_DIR, 70, 15);
        ext_flat(INV_DIR, 80, 20);
        ext_flat(INV_DIR, 90, 20);
        ext_flat(INV_DIR, 110, 20);
        ext_flat(INV_DIR, SLOW_PASS, 15);
        ext_flat(INV_DIR, TURN_PASS, 10);

        ext_flat(NOM_DIR, TURN_PASS, 10);
        ext_flat(NOM_DIR, SLOW_PASS, 15);
        ext_flat(NOM_DIR, 110, 20);
        ext_flat(NOM_DIR, 90, 20);
        ext_flat(NOM_DIR, 80, 20);
        ext_flat(NOM_DIR, 70, 15);
        ext_flat(NOM_DIR, FAST_PASS, 0);
        ext_flat(NOM_DIR, 70, 15);
        ext_flat(NOM_DIR, 80, 20);
        ext_flat(NOM_DIR, 90, 20);
        ext_flat(NOM_DIR, 110, 20);
        ext_flat(NOM_DIR, SLOW_PASS, 10);
        ext_flat(NOM_DIR, TURN_PASS, 15);
      }
      l_turn = resrv2;//
      for (int i = 0; i<l_turn; i++ ) { ////20//resrv2
        ext_flat(INV_DIR, TURN_PASS, 15);
        ext_flat(INV_DIR, SLOW_PASS, 10);
        ext_flat(INV_DIR, 110, 20);
        ext_flat(INV_DIR, 90, 20);
        ext_flat(INV_DIR, 80, 20);
        ext_flat(INV_DIR, 70, 15);
        if (i == 10) {
          ext_flat(INV_DIR, FAST_PASS, 170 + 7500);
        }
        else if ( i == 17) {
          ext_flat(INV_DIR, FAST_PASS, 170 + 500);
        }
        else if ( i == 28) {
          ext_flat(INV_DIR, FAST_PASS, 170 + 500);
        }
        else if ( i == 35) {
          ext_flat(INV_DIR, FAST_PASS, 170 + 500);
        }
        else {
          ext_flat(INV_DIR, FAST_PASS, 170);
        }
        ext_flat(INV_DIR, 70, 15);
        ext_flat(INV_DIR, 80, 20);
        ext_flat(INV_DIR, 90, 20);
        ext_flat(INV_DIR, 110, 20);
        ext_flat(INV_DIR, SLOW_PASS, 15);
        ext_flat(INV_DIR, TURN_PASS, 10);

        ext_flat(NOM_DIR, TURN_PASS, 10);
        ext_flat(NOM_DIR, SLOW_PASS, 15);
        ext_flat(NOM_DIR, 110, 20);
        ext_flat(NOM_DIR, 90, 20);
        ext_flat(NOM_DIR, 80, 20);
        ext_flat(NOM_DIR, 70, 15);
        ext_flat(NOM_DIR, FAST_PASS, 170);
        ext_flat(NOM_DIR, 70, 15);
        ext_flat(NOM_DIR, 80, 20);
        ext_flat(NOM_DIR, 90, 20);
        ext_flat(NOM_DIR, 110, 20);
        ext_flat(NOM_DIR, SLOW_PASS, 10);
        ext_flat(NOM_DIR, TURN_PASS, 15);
      }
      ext_flat(NOM_DIR, 200,8700);//8900
      ext_flat(NOM_DIR, 300,300);
      ext_flat(NOM_DIR, 100*27,100);//resrv3 23

      ext_flat(INV_DIR, 300, 12);        
      ext_flat(INV_DIR, 250, 12);
      ext_flat(INV_DIR, 200, 10);
      ext_flat(INV_DIR, 170, 10);
      ext_flat(INV_DIR, 120, 10);
      ext_flat(INV_DIR, 100, 15);
      ext_flat(INV_DIR, 80, 20);
      ext_flat(INV_DIR, 70, 30);
      ext_flat(INV_DIR, 60, 50);
      ext_flat(INV_DIR, 50, 70);
      ext_flat(INV_DIR, 45, 60);
      ext_flat(INV_DIR, 45, 39000);

      pitta_set_temp((temp_temp_extruder), 0);   
      ext_flat(INV_DIR, 45/* +pitta_extrude_return_spd */, 20000);//
      ext_flat(INV_DIR, 60/* +pitta_extrude_return_spd */, 10000);//  

    }
    break;
  }

   
  E0_STEP_WRITE(LOW);
  delayMicroseconds(30);
  pitta_wtcdog_reset();
  b_snap_done = true;
  b_req_retract_fully = true;
  pitta_ui_thermal_update();

  long int wait_expire_cnt = 0;
  bool b_step = false;
  // mExtruder_dir(INV_DIR);
  int termal_expire_cnt = 0;
  bool b_retract_more_req = false;
  b_retract_more_req = false;
  int reset_ext_driver_cnt = 0;
  long int wait_expire_lim = 300000;

  if (b_stop_active) {
    // wait_expire_lim = 70000;
    wait_expire_lim = tb_len*100;
    if (wait_expire_lim<45000) wait_expire_lim = 45000;
  }
  else {
    wait_expire_lim = 300000;
  }
  wait_expire_cnt = 0;

  while (chk_material() && wait_expire_cnt < wait_expire_lim ) {//130000
    b_retract_more_req = true;
    wait_expire_cnt++;
    reset_ext_driver_cnt++;
    if (reset_ext_driver_cnt>40000) {
      reset_ext_driver_cnt = 0;
      if (!b_pitta_jammed) {
        E0_ENABLE_WRITE(HIGH);
        delay(50);
        E0_ENABLE_WRITE(LOW);
        delay(50);
      }
    }
    // if (wait_expire_cnt>40000) {

    // }

    b_step = !b_step;
    {
      if (b_step)
      {
        E0_STEP_WRITE(HIGH);
      }
      else
      {
        E0_STEP_WRITE(LOW);
      }    
    }
    delayMicroseconds(70);
    termal_expire_cnt++;
    if (termal_expire_cnt>100) {
      termal_expire_cnt = 0;
      pitta_req_manage_heater_update();
      pitta_wtcdog_reset();
    }
  }

  // delay(2000);

  for (int i = 0; i<16*1000;i++) {//resrv4:16
    b_step = !b_step;
    {
      if (b_step)
      {
        E0_STEP_WRITE(HIGH);
      }
      else
      {
        E0_STEP_WRITE(LOW);
      }    
    }
    delayMicroseconds(120);
    termal_expire_cnt++;
    if (termal_expire_cnt>100) {
      termal_expire_cnt = 0;
      pitta_req_manage_heater_update();
      pitta_wtcdog_reset();
    }
  }
  pitta_req_manage_heater_update();
  mExtruder_dir(NOM_DIR);
  for (int i = 0; i < 1800; i++) {
    b_step = !b_step;
    if (b_step)
    {
      E0_STEP_WRITE(HIGH);
    }
    else
    {
      E0_STEP_WRITE(LOW);
    }
    delayMicroseconds(120);
  } 
  pitta_req_manage_heater_update();
  mExtruder_dir(NOM_DIR);
}

void retract_ready()
{
  bool b_step = false;
  bool b_material_empty = false;
  int chk_cnt = 0;
  pitta_enable_e0();
  pitta_wtcdog_reset();

  long wait_expire_cnt = 0;
  SET_INPUT_PULLDOWN(ONE_W_CMD_PIN);
  delay(5);
  cur_ext_dir = NOM_DIR;
  E0_DIR_WRITE(INVERT_E0_DIR ? !NOM_DIR : NOM_DIR);
  delay(1);
  mExtruder_dir(NOM_DIR);
  safe_delay(50);
  while (!b_material_empty && wait_expire_cnt < 2000*MUL_V) {
    if (!chk_material()) {
      chk_cnt++;
      if (chk_cnt>200) {
        b_material_empty = true;
        chk_cnt = 0;
      }
    }
    b_step = !b_step;
    // mExtruder_dir(b_mot_dir);
    if (b_step)
    {
      E0_STEP_WRITE(HIGH);
    }
    else
    {
      E0_STEP_WRITE(LOW);
    }    
    delayMicroseconds(150/WAIT_MUL_V);
    pitta_wtcdog_reset();
    wait_expire_cnt++;
  }
  delay(5);
  mExtruder_dir(INV_DIR);
  delay(5);
  wait_expire_cnt = 0;
  int termal_expire_cnt = 0;
  chk_cnt = 0;
  SERIAL_ECHOLNPGM("PITTA ready retracting and sensor waiting ");
  b_material_empty = false;
  while (!b_material_empty && wait_expire_cnt < 130000) {//40000//1800000
    if (!chk_material()) {
      chk_cnt++;
      if (chk_cnt>2500) {
        b_material_empty = true;
        chk_cnt = 0;
      }
    }    
    delayMicroseconds(150/WAIT_MUL_V);
    pitta_wtcdog_reset();
    wait_expire_cnt++;
    termal_expire_cnt++;
    if (termal_expire_cnt > 200) {
      termal_expire_cnt = 0;
      pitta_req_manage_heater_update();
    }
    b_step = !b_step;
    // mExtruder_dir(b_mot_dir);
    if (b_step)
    {
      E0_STEP_WRITE(HIGH);
    }
    else
    {
      E0_STEP_WRITE(LOW);
    }       
  }  
  long retract_more_cnt = 10000*MUL_V;
  while (retract_more_cnt > 0)
  {
    // chk_material();
    retract_more_cnt--;
    b_step = !b_step;
    // mExtruder_dir(b_mot_dir);
    if (b_step)
    {
      E0_STEP_WRITE(HIGH);
    }
    else
    {
      E0_STEP_WRITE(LOW);
    }
    delayMicroseconds(90);
    pitta_wtcdog_reset();
  }
  delay(5);
  mExtruder_dir(NOM_DIR);
  delay(5);
  PITTA::b_retract_ready = true;
  // SERIAL_ECHOLNPGM("PITTA ready done ");
}

#define BUFFER_TOP 3
long send_buffer[BUFFER_TOP] = {0, 0, 0};
int buffer_top = 0, buffer_bottom = 0;
int elem_consume_pos = 0;
int elem_accum_pos = 0;
int carry_pos = 0;
int buffer_diff = 0;

bool add_data_to_buffer(long data)
{
  elem_accum_pos++;
  if (elem_accum_pos == BUFFER_TOP)
  {
    carry_pos = carry_pos + BUFFER_TOP;
    elem_accum_pos = 0;
  }
  buffer_diff = elem_accum_pos - elem_consume_pos + carry_pos;
  send_buffer[elem_accum_pos] = data;
  return true;
}
#define CMD_DIST_PITTA_READY_REQ 0x0010
extern bool b_pitta_print_ready_req;
long get_data_from_buffer()
{
  elem_consume_pos++;
  if (elem_consume_pos == BUFFER_TOP)
  {
    carry_pos = carry_pos - BUFFER_TOP;
    elem_consume_pos = 0;
  }
  buffer_diff = elem_accum_pos - elem_consume_pos + carry_pos;
  return send_buffer[elem_consume_pos];
}



PITTA::PITTA()
{
  rx_buffer[0] = '\0';
}

void PITTA::enable(bool b_enable)
{
  pitta_enabled = b_enable;
  b_retract_ready = false;
}

#ifdef MODEL_E3V2_PRO32
void PITTA::init()
{
  set_runout_valid(false);
  change_turn_val = 0;

#if PIN_EXISTS(PITTA_CMD)
  WRITE(ONE_W_CMD_PIN, HIGH);
  SET_OUTPUT(ONE_W_CMD_PIN);
#endif

  extruder = PITTA_NO_TOOL;

  safe_delay(10);
  reset();
  rx_buffer[0] = '\0';
  pitta_state = 1;

  HAL_timer_start(MF_TIMER_PITTA, 122);
}
#endif

#ifdef MODEL_E3PRO
void PITTA::init()
{

  set_runout_valid(false);
  change_turn_val = 0;

#if PIN_EXISTS(PITTA_CMD)
  WRITE(PITTA_CMD_PIN, HIGH);
  SET_OUTPUT(PITTA_CMD_PIN);
#endif

  extruder = PITTA_NO_TOOL;

  safe_delay(10);
  reset();
  rx_buffer[0] = '\0';
  pitta_state = 1;
}
#endif



void PITTA::pitta_stop()
{
  b_stop_active = true;
  ext_snap();
  b_stop_active = false;
  // disable_E0();
}

void PITTA::reset()
{
}

uint8_t PITTA::get_current_tool()
{
  return extruder == PITTA_NO_TOOL ? -1 : extruder;
}

/**
 * PITTA main loop - state machine processing
 */

typedef enum pitta_data_process {LISTEN = 0, RECEIVING, PARSER, PHY_PROCESS, PREPAIR, SENDING, STATE_PROCESS } pitta_data_process;
pitta_data_process pitta_data_state, prev_pitta_data_state;

// bool b_req_phy_process = false;
bool b_pitta_ui_force_update_req = false;
bool b_change_done = true;
bool b_selector_done = true;
long resend_var = 0;
bool b_resend_req = false;
// int resend_retry_cnt = 0;
bool b_rcv_confirm_need = false;
long rcv_packet_expire = 0;
int rcv_bit_expire_cnt = 0;

static bool b_wire_sent_byte = false;


bool b_master_say = true, b_master_hear = false;
bool b_master_state_change_request = false;
bool b_master_state_changing = true;
bool b_update_M_send_data = false;

bool b_receive_tick = false, b_pitta_data_init = false, b_receive_start = false, b_byte_prepaired = false;
int m_wait_cnt = 0;
int receive_tick_hi_cnt_sum = 0;
int receive_tick_lo_cnt_sum = 0;
bool b_master_receiving = false;
bool b_pitta_sending = false;
bool b_check_wait = false;
bool b_ware_spreading = false;
unsigned int recv_lock_release_cnt = 0;

void PITTA::send_force_cmd(int cmd)
{
  b_resend_req = true;
  resend_retry_cnt++;
  resend_var = cmd;
  b_master_state_change_request = true;
  b_master_say = true;
  b_master_hear = false;
}

bool b_state_proceed = true;
extern Planner planner;
bool b_test_com_enabled = false;
unsigned int ware_expire_cnt = 0;

void PITTA::pitta_act() {
  while (pitta.pitta_sel_req) 
  {    
    pitta.pitta_loop();
  }
}


void PITTA::pitta_loop()
{
  prev_main_sys_tick = main_sys_tick;
  main_sys_tick = PITTA_MAIN_TICK;

  if (prev_main_sys_tick > main_sys_tick)
  {
    loop_diff_tick = main_sys_tick + (0xffff - prev_main_sys_tick);
  }
  else
  {
    loop_diff_tick = (main_sys_tick - prev_main_sys_tick);
  }

  if (loop_diff_tick > REF_CLK_BASE*CLK_MOD)
    loop_diff_tick = REF_CLK_BASE*CLK_MOD; //5000 for pro
  if (b_state_proceed)
  {
    switch (pitta_state)
    {
    case 0: // Disabled
      break;
    case 1: // Init
      if (pitta_start())
      {
        prev_FS_request = millis(); // Initialize filament sensor timeout

        SERIAL_ECHOLNPGM("PITTA start");
        pitta_state = 2;
      }
      else if (millis() > 30000)
      { // 30sec
        SERIAL_ECHOLNPGM("PITTA not responding - DISABLED");
        pitta_state = 0;
      }
      break;
    case 2: // Get Version
      if (pitta_ok())
      {
        SERIAL_ECHOLNPGM("PITTA version ok");
        pitta_state = 3;
      }
      break;
    case 3: // Get Filament Sensor
      if (pitta_ok())
      {
        SERIAL_ECHOLNPGM("PITTA enabled");
        pitta_enable_e0();
        pitta_enabled = true;
        pitta_state = 4;
      }
      break;
    case 4: // Idle
      if (cmd)
      {
        if (WITHIN(cmd, PITTA_CMD_T0, PITTA_CMD_T7))
        {
          // tool change
          int filament = cmd - PITTA_CMD_T0; // filament index[0..7]
          SERIAL_ECHOLNPGM("PITTA T", filament);
          b_master_state_change_request = true;
          b_master_hear = false;
          b_master_say = true;
          receive_tick_hi_cnt = 0;
          pitta_state = 6; // wait response
        }
        else if (WITHIN(cmd, PITTA_CMD_L0, PITTA_CMD_L7))
        {
          // load filament
          int filament = cmd - PITTA_CMD_L0;
          SERIAL_ECHOLNPGM("PITTA L", filament);
          pitta_state = 6; // wait response
        }
        else if (cmd == PITTA_CMD_C)
        {
          SERIAL_ECHOLNPGM("PITTA C");
          pitta_state = 6; // wait response
        }
        else if (cmd == PITTA_CMD_R)
        {
          // retract current filament
          SERIAL_ECHOLNPGM("PITTA R");
          pitta_state = 6; // wait response
        }
        else if (cmd == PITTA_CMD_E)
        {
          // extrude current filament
          SERIAL_ECHOLNPGM("PITTA E");
          pitta_state = 6; // wait response
        }
        last_cmd = cmd;
        cmd = PITTA_CMD_NONE;
      }
      else if (ELAPSED(millis(), prev_FS_request + 3000))
      {
        pitta_state = 5;
      }
      break;
    case 5: // response to FS command
      if (pitta_ok())
      {
        fila_sens = 1; // TEST
        if (!fila_sens && fila_runout_valid)
          filament_runout();
        if (cmd == PITTA_CMD_NONE) {
          pitta_ready = true;
          // jam_expire_cnt = 0;
        }
        pitta_state = 4;
      }
      else if (ELAPSED(millis(), prev_request + PITTA_FS_TIMEOUT))
      {
        pitta_state = 4;
      }
      break;
    case 6: // response to pitta commands
      if (b_change_done)
      {
        constexpr bool keep_trying = false;

        if (!keep_trying)
        {
          pitta_ready = true;
          pitta_state = 4;
          last_cmd = PITTA_CMD_NONE;
        }
      }
      break;
    default:
      break;
    }
    b_state_proceed = false;
  }


#define HEAR_LOW_RDY_DTY_THR 1000

  static unsigned int test_watch_cnt = 0, thermal_cont_cnt = 0;
  test_watch_cnt++;
  if (test_watch_cnt>500) {//2000
    test_watch_cnt = 0;
    pitta_wtcdog_reset();
    pitta_req_manage_heater_update();
    thermal_cont_cnt++;    
    if (pitta_data_state!=RECEIVING&&pitta_data_state!=SENDING) {
      // pitta_req_manage_heater_update();
      if (thermal_cont_cnt>30) {//30
        thermal_cont_cnt = 0;
        pitta_ui_update();
        // SERIAL_ECHOLNPGM("UI and Thermal force update");
      }
    }
  }

  static unsigned int ware_wait_cnt = 0;
  
  if (b_ware_spreading) {
    if (ware_expire_cnt<5000) {
      if (cur_ext_dir == NOM_DIR) {
        delay(8);
        E0_STEP_WRITE(LOW);
        delay(10);
        mExtruder_dir(INV_DIR);
        delay(10);
      }
      ware_wait_cnt++;
      if (ware_wait_cnt > 500) {
        ware_expire_cnt++;
        ware_wait_cnt = 0;
        static bool b_step = false;
          b_step = !b_step;
          // mExtruder_dir(b_mot_dir);
          if (b_step)
          {
            E0_STEP_WRITE(HIGH);
            jam_expire_cnt++;
          }
          else
          {
            E0_STEP_WRITE(LOW);
          }
          // delay(2);
      }
    }
  }

  prev_pitta_data_state = pitta_data_state;
  switch (pitta_data_state) {
    
    case LISTEN:
      on_listen();
      break;
    case RECEIVING:
      on_receiving();
      break;
    case PARSER:
      parsing();
      break;
    case PHY_PROCESS:
      physical_processing();
      break;
    case PREPAIR:
      data_prepair();
      break;      
    case SENDING:
      sending();
      break;
    case STATE_PROCESS:
      state_processing();
      break;  
    default:
      pitta_data_state = LISTEN;
  }
  if (prev_pitta_data_state != pitta_data_state) {
    rcv_packet_expire = 0;
  }
}


/**
 * Check if the data received ends with the given string.
 */
bool PITTA::rx_str_P(const char *str)
{
  uint8_t i = strlen(rx_buffer);

  rx_buffer[i] = '\0';

  return true;
}

/**
 * Transfer data to PITTA, no argument
 */
void PITTA::tx_str_P(const char *str)
{
  clear_rx_buffer();
  uint8_t len = strlen_P(str);

  prev_request = millis();
}

/**
 * Transfer data to PITTA, single argument
 */
void PITTA::tx_printf_P(const char *format, int argument = -1)
{
  clear_rx_buffer();
  uint8_t len = sprintf_P(tx_buffer, format, argument);

  prev_request = millis();
}

/**
 * Empty the rx buffer
 */
void PITTA::clear_rx_buffer()
{
  rx_buffer[0] = '\0';
}

/**
 * Check PITTA was started
 */
bool PITTA::pitta_start()
{
  // Check start message
  int8_t res = rx_str_P(PSTR("start\n"));
  return res;
}

/**
 * Check if we received 'ok' from PITTA
 */
bool PITTA::pitta_ok()
{
  if (rx_str_P(PSTR("ok\n")))
  {
    prev_FS_request = millis();
    return true;
  }
  return false;
}

void PITTA::command(const uint8_t command)
{
  if (!pitta_enabled)
    return;
  cmd = command;
  pitta_ready = false;
  // pitta_sel_req = true;
}

/**
 * Wait for response from PITTA
 */
bool PITTA::get_response()
{
  while (cmd != PITTA_CMD_NONE)
  {
    idle();
  }

  while (!pitta_ready)
  {
    idle();
    if (pitta_state != 6)
      break;
  }

  const bool ret = pitta_ready;
  pitta_ready = false;

  return ret;
}

/**
 * Wait for response and deal with timeout if necessary
 */
void PITTA::manage_response(const bool move_axes, const bool turn_off_nozzle)
{
  bool response = false;
  pitta_print_saved = false;

  // while (!b_change_done) {
  //   safe_delay(10);
  // }

  while (!response)
  {

    response = get_response(); // wait ok from pitta
    if (!response)
    { // No "ok" was received in reserved time frame, user will fix the issue on pitta unit

      if (!pitta_print_saved)
      { // First occurrence. Save current position, park print head, disable nozzle heater.

        if (!b_test_com_enabled) pitta_planner_sync();

        pitta_print_saved = true;

        SERIAL_ECHOLNPGM("PITTA not responding");
      }
    }
    else if (pitta_print_saved)
    {
      SERIAL_ECHOLNPGM("PITTA starts responding");
    }
  }
}

void PITTA::filament_runout()
{
  //enqueue_and_echo_commands_P(PSTR("M600"));
  if (!b_test_com_enabled) pitta_planner_sync();
}


void PITTA::e0_state_set(bool dir, int spd)
{
  PITTA::b_e0_dir = dir;
  PITTA::e0_spd = spd;

  E0_ENABLE_WRITE(LOW);
  E0_DIR_WRITE(INVERT_E0_DIR ? !(PITTA::b_e0_dir) : (PITTA::b_e0_dir));
}

bool b_first_change = false;
bool b_pitta_print_ready_req = false;

void sync_remain_one()
{
  while (abs(Planner::block_buffer_head - Planner::block_buffer_tail) > 0)
    idle();
}



void PITTA::pitta_state_proceed() {
  // PITTA State Process

  if (b_state_proceed)
  {
    switch (pitta_state)
    {
    case 0: // Disabled
      break;
    case 1: // Init
      if (pitta_start())
      {
        prev_FS_request = millis(); // Initialize filament sensor timeout

        SERIAL_ECHOLNPGM("PITTA start");
        pitta_state = 2;
        b_pitta_just_enabled = false;
      }
      else if (millis() > 30000)
      { // 30sec
        SERIAL_ECHOLNPGM("PITTA not responding - DISABLED");
        pitta_state = 0;
      }
    //   break;
    // case 2: // Get Version
      if (pitta_ok())
      {
        SERIAL_ECHOLNPGM("PITTA version ok");
        pitta_state = 3;
      }
      // break;
    // case 3: // Get Filament Sensor
      if (pitta_ok())
      {
        SERIAL_ECHOLNPGM("PITTA enabled");
        pitta_enable_e0();
        pitta_enabled = true;
        b_pitta_just_enabled = true;
        pitta_state = 4;
      }
      break;
    case 4: // Idle
      if (cmd)
      {
        if (WITHIN(cmd, PITTA_CMD_T0, PITTA_CMD_T7))
        {
          // tool change
          int filament = cmd - PITTA_CMD_T0; // filament index[0..7]
          SERIAL_ECHOLNPGM("PITTA T", filament);
          b_master_state_change_request = true;
          b_master_hear = false;
          b_master_say = true;
          receive_tick_hi_cnt = 0;
          pitta_state = 6; // wait response
        }
        else if (WITHIN(cmd, PITTA_CMD_L0, PITTA_CMD_L7))
        {
          // load filament
          int filament = cmd - PITTA_CMD_L0;
          SERIAL_ECHOLNPGM("PITTA L", filament);
          pitta_state = 6; // wait response
        }
        else if (cmd == PITTA_CMD_C)
        {
          SERIAL_ECHOLNPGM("PITTA C");
          pitta_state = 6; // wait response
        }
        else if (cmd == PITTA_CMD_R)
        {
          // retract current filament
          SERIAL_ECHOLNPGM("PITTA R");
          pitta_state = 6; // wait response
        }
        else if (cmd == PITTA_CMD_E)
        {
          // extrude current filament
          SERIAL_ECHOLNPGM("PITTA E");
          pitta_state = 6; // wait response
        }
        last_cmd = cmd;
        cmd = PITTA_CMD_NONE;
      }
      break;
    case 6: // response to pitta commands
      if (b_change_done)
      {
        constexpr bool keep_trying = false;

        if (!keep_trying)
        {
          pitta_ready = true;
          pitta_state = 4;
          last_cmd = PITTA_CMD_NONE;
        }
      }
      break;
    default:
      break;
    }
    b_state_proceed = false;
  }
}


/**
 * Handle tool change
 */
// int16_t temp_temp_extruder = 0, temp_temp_bed = 0;
int nozzle_turn = 0;
int g_index = 0;
void PITTA::fila_change(const uint8_t index)
{
  extern bool b_state_proceed;
  b_state_proceed = true;
  b_pitta_jammed = false;
  jam_expire_cnt = 0;
  resend_retry_cnt = 0;
  change_talk_expire_cnt = 0;
  g_index = index;

  if (!pitta_enabled) {
    pitta_state_proceed();
  }

  {
    {
      if (!b_test_com_enabled) pitta_planner_sync();
      pitta_sel_req = true;
      b_pitta_loop_lock = true;
      b_pitta_lock = true;
      SERIAL_ECHOLNPGM("send_lock");

#ifdef MODEL_E3PRO
      lcd_status_printf_P(0, PSTR("Chg F%i & turn %i"), int(index + 1), int(change_turn_val));
#endif
#ifdef MODEL_E3V2_PRO32

#ifdef DEV_NCMD_PRINT
      ui.status_printf(0, F("PE:%i CE:%i TB_L:%i"), int(wrong_packet_cnt), int(wrong_cmd_cnt), int(tb_len));
#else
      // ui.status_printf(0, F("Chg F%i & turn %i"), int(index + 1), int(change_turn_val));
      ui.status_printf(0, F("F:%i N:%i TB_LEN:%i"), int(g_index + 1), int(change_turn_val), int(tb_len));
#endif 
#endif
      if (b_pitta_just_enabled) {
        // b_pitta_just_enabled = false;
        nozzle_turn = 0;
        add_data_to_buffer(((index<<8)|RDY_REQ));//
        SERIAL_ECHOLNPGM("rdy_req queud");
      }
      else {
        add_data_to_buffer(((index<<8)|DATA_W_NO_CMD));
      }
      nozzle_turn++;
      SERIAL_ECHOLNPGM("queuing: ", index);
      extruder = index; // filament change is finished
      // SERIAL_ECHOLNPGM("extruder: ", extruder);
      active_extruder = 0;
      e0_state_set(true, 100);
      temp_temp_extruder = pitta_get_temp(0);
      temp_temp_bed = pitta_get_temp_bed();
      pitta_ui_update();
      if (temp_temp_extruder < 0)
      {
        SERIAL_ECHOLNPGM("cold temp_temp_extruder: ", temp_temp_extruder);
        temp_temp_extruder = 0;
      }
      SERIAL_ECHOLNPGM("temp_temp_extruder: ", temp_temp_extruder);
      pitta_set_temp((temp_temp_extruder - 0), 0);
      
      if (b_pitta_just_enabled) {
        while(pitta_get_celsius(0)<185) {
          delay(7);
          pitta_wtcdog_reset();
          pitta_ui_thermal_update();
        }
        b_pitta_just_enabled = false; 
        retract_ready();
        SERIAL_ECHOLNPGM("material retract ready");
        confirmed_set_output_low();
        pitta_data_state = PREPAIR;        
      }
      else {
        SET_INPUT_PULLUP(ONE_W_CMD_PIN);
        ext_snap();
        delay(150);
        pitta_data_state = PREPAIR;
        change_turn_val++;        
      }
      
      b_material_detected_from_pitta = false;
      pitta_set_temp(temp_temp_extruder, 0);

      // ui.status_printf_P(0, PSTR("Changing F%i..."), int(index + 1));
      b_change_done = false;
      extrude_init_dist = 0;
      command(PITTA_CMD_T0 + index);
      manage_response(true, true);
      b_selector_done = false;
      #ifdef MODEL_E3V2_PRO32
      ui.reset_status();
      #endif
      #ifdef MODEL_E3PRO
      lcd_reset_status();
      #endif
      SERIAL_ECHOLN("");    
      SERIAL_ECHO("turn : ");
      SERIAL_ECHOLN(nozzle_turn);    
      SERIAL_ECHO("slot : ");
      SERIAL_ECHOLN(extruder);
      SERIAL_ECHOLN("");         
      change_talk_expire_cnt = 0;
      pitta_req_manage_heater_update();
      b_ware_spreading = true;
      delay(1);
      mExtruder_dir(INV_DIR);
      delay(1);
    }
  }
}

/**
 * Unload from hotend and retract to MMU
 */
bool PITTA::unload()
{
  if (!pitta_enabled)
    return false;

  if (thermalManager.tooColdToExtrude(active_extruder))
  {
    return false;
  }

  // Unload sequence to optimize shape of the tip of the unloaded filament
  // execute_extruder_sequence((const E_Step *)ramming_sequence, sizeof(ramming_sequence) / sizeof(E_Step));

  // no active tool
  extruder = PITTA_NO_TOOL;

  return true;
}

void PITTA::on_listen() {


  abs_comm_receive_remain_tick = abs_comm_receive_remain_tick + loop_diff_tick;
  if (abs_comm_receive_remain_tick > ABS_COMM_RECEIVE_CNT_DUTY)
  {
    rcv_packet_expire++;
    if (rcv_packet_expire>25000) {//40000 //90000
      if (b_rcv_confirm_need) {
        SERIAL_ECHOLN("");
        SERIAL_ECHOLN("Listen expire, Retry Request");
        b_pitta_data_init = false;
        b_resend_req = true;
        resend_retry_cnt++;
        wait_for_out_set_possible();       
        // SERIAL_ECHOLN("move from LISTEN to PREPAIR for resend request");
        // SET_OUTPUT(ONE_W_CMD_PIN);
        // WRITE(ONE_W_CMD_PIN, LOW);
        confirmed_set_output_low();
        pitta_data_state = PREPAIR;
        SERIAL_ECHOLN("go to PREPAIR");
        SERIAL_ECHOLN("");
      }
      rcv_packet_expire = 0;
    }

    abs_comm_receive_remain_tick = abs_comm_receive_remain_tick - ABS_COMM_RECEIVE_CNT_DUTY;
    b_state_proceed = true;
    b_receive_tick = READ(ONE_W_CMD_PIN);

    if (b_receive_tick)
    {
      receive_tick_hi_cnt++;
      if (receive_tick_hi_cnt > 30000)
      {
        receive_tick_hi_cnt = 28000;
      }
      receive_tick_lo_cnt_sum = receive_tick_lo_cnt;
      receive_tick_lo_cnt = 0;
      b_pitta_data_init = false;
    }
    else
    {
      receive_tick_lo_cnt++;
      if (receive_tick_lo_cnt > 30000)
      {
        receive_tick_lo_cnt = 25000;
      }
      receive_tick_hi_cnt_sum = receive_tick_hi_cnt;
      receive_tick_hi_cnt = 0;
    }
    if (!b_receive_tick) {
      b_ok_ready_to_say = false;
      must_ready_hear_cnt++;
      if (must_ready_hear_cnt>HEAR_LOW_RDY_DTY_THR) {
        must_ready_hear_cnt = 0;
        b_on_hearing = true;
        receive_tick_hi_cnt_sum = 0;
        receive_tick_hi_cnt = 0;
        receive_tick_lo_cnt_sum = 0;
        receive_tick_lo_cnt = 0;
        SERIAL_ECHOLNPGM("move from LISTEN to RECEIVING");
        SET_INPUT_PULLUP(ONE_W_CMD_PIN);    
        pitta_data_state = RECEIVING;
        rcv_packet_expire = 0;
      }
    }
  }
}



void PITTA::on_receiving() {
  static long /* received_cmd8, received_low8, received_high8, */ received_chk8;
  
  abs_comm_receive_remain_tick = abs_comm_receive_remain_tick + loop_diff_tick;
  if (abs_comm_receive_remain_tick > ABS_COMM_RECEIVE_CNT_DUTY)
  {
    rcv_packet_expire++;
    if (rcv_packet_expire>5000) {//30000
      if (b_rcv_confirm_need) {
        SERIAL_ECHO("Rcv: ");
        SERIAL_ECHOLN((unsigned)receive_byte);
        SERIAL_ECHOLN("Retry Request expire");
        receive_byte = 0;
        b_pitta_data_init = false;
        b_resend_req = true;
        resend_retry_cnt++;
        SERIAL_ECHOLN("move from RCV to PREPAIR for retry request");
        confirmed_set_output_low();
        pitta_data_state = PREPAIR;
        pitta_req_manage_heater_update();
        rcv_packet_expire = 0;
        return;        
      }
      else {
        SERIAL_ECHOLN("move from RCV to LISTEN, expire, ignore packet");
        receive_byte = 0;
        SET_INPUT_PULLUP(ONE_W_CMD_PIN);
        pitta_data_state = LISTEN;  
        receive_byte = 0;
        receive_tick_hi_cnt_sum = 0;
        receive_tick_hi_cnt = 0;
        receive_tick_lo_cnt_sum = 0;
        receive_tick_lo_cnt = 0;
        receive_bit_shift = 0;  
        pitta_req_manage_heater_update();
        rcv_packet_expire = 0;
        return;        
      }         
    }

    abs_comm_receive_remain_tick = abs_comm_receive_remain_tick - ABS_COMM_RECEIVE_CNT_DUTY;
    b_state_proceed = true;
    b_receive_tick = READ(ONE_W_CMD_PIN);

    if (b_receive_tick)
    {
      receive_tick_hi_cnt++;
      if (receive_tick_hi_cnt > 10000)
      {
        receive_tick_hi_cnt = 5000;
      }
      receive_tick_lo_cnt_sum = receive_tick_lo_cnt;
      receive_tick_lo_cnt = 0;
      b_pitta_data_init = false;
    }
    else
    {
      receive_tick_lo_cnt++;
      if (receive_tick_lo_cnt > 10000)
      {
        receive_tick_lo_cnt = 5000;
      }
      receive_tick_hi_cnt_sum = receive_tick_hi_cnt;
      receive_tick_hi_cnt = 0;
    }


    if (receive_tick_hi_cnt_sum>0) {
      if (!b_receive_start  /*  && receive_bit_shift > 0  */ ) {//2
          b_receive_start = true;
      }
      if (receive_tick_hi_cnt_sum>10) {// 9// 40 for pitta slave
        receive_byte = receive_byte | ((long)1 << receive_bit_shift);
      }
      else {
      }
      receive_tick_hi_cnt_sum = 0;
      receive_bit_shift++;
      rcv_packet_expire = 0;
      rcv_bit_expire_cnt = 0;
      if (receive_bit_shift == 32) {
          // SERIAL_ECHOLN(" ");
          b_pitta_data_init = false;
          b_receive_start = false;
          b_byte_prepaired = true;
          received_packet = receive_byte;
          received_cmd = 0xff & receive_byte;
          received_data = (0xffff00 & receive_byte) >> 8;
          if (received_data>32767) {
            received_data = received_data - 65536;
          }
          received_chk8 = (0xff000000 & receive_byte) >> 24;
          long chk_high8 = 0;
          for (long i = 0;i<24; i++) {
              chk_high8 = chk_high8 + (((0xffffff & receive_byte)>>i)&0x1)*(i+0)*2;
          }
          chk_high8 = chk_high8 & 0xff;
          SERIAL_ECHOLNPGM("rcv chk: ", (unsigned)received_chk8);

          if (chk_high8 == received_chk8&&received_packet!=0) {
              b_byte_receive_done = true;
              SERIAL_ECHO("Rcv: ");
              SERIAL_ECHOLN(received_data);
              SERIAL_ECHOLN((unsigned)received_packet);
              b_resend_req = false;
              resend_retry_cnt = 0;
              SERIAL_ECHOLN("move from RCV to PARSER");
              pitta_data_state = PARSER;              
          }
          else {
              SERIAL_ECHO("Rcv: ");
              SERIAL_ECHOLN(received_data);
              SERIAL_ECHOLN((unsigned)received_packet);
              SERIAL_ECHOLN("Retry Request H");
              if (received_packet == 0) {
                #ifdef DEV_NCMD_PRINT
                wrong_packet_cnt++;
                #endif
                SERIAL_ECHOLN("Wrong command Received!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
              }
              b_pitta_data_init = false;
              b_resend_req = true;
              resend_retry_cnt++;
              SERIAL_ECHOLN("move from RCV to PREPAIR for retry request");
              confirmed_set_output_low();
              pitta_data_state = PREPAIR;
          }
          received_packet = 0;
          receive_byte = 0;
          receive_tick_hi_cnt_sum = 0;
          receive_tick_hi_cnt = 0;
          receive_tick_lo_cnt_sum = 0;
          receive_tick_lo_cnt = 0;
          receive_bit_shift = 0;
          pitta_req_manage_heater_update();
      }      
    }
  }
}




void PITTA::parsing() {
  b_state_proceed = true;
  pitta_req_manage_heater_update();
  {
    if (received_cmd == RESEND_REQ) {
      send_packet = 0xffffffff & resend_var;
      SERIAL_ECHOLNPGM("RETRY: ", send_packet);
      long chk_high8 = 0;
      for (int i = 0;i<24; i++) {
          chk_high8 = chk_high8 + (((0xffffff & send_packet)>>i)&0x1)*(i+0)*2;
      }
      chk_high8 = chk_high8 & 0xff;
      send_packet = send_packet|(chk_high8<<24);
      // SERIAL_ECHOLNPGM("chk sum: ", chk_high8);
      // SERIAL_ECHOLNPGM("sent data: ", send_packet);
      b_update_M_send_data = true;
      b_start_send_bit = true;
      b_pitta_sending = true;
      send_bit_shift = 0;
      io_chk_n_set_as_output(ONE_W_CMD_PIN);
      WRITE(ONE_W_CMD_PIN, LOW);  
      safe_delay(500);//800//300 
      pitta_data_state = SENDING;  
      pitta_wtcdog_reset();
    }
    else if (received_cmd == CHG_DONE ) {
      proc_state = PROC_CHANGE_DONE;
      SERIAL_ECHOLN("CHG_DONE received, go to extrude");
      material_offset = (long)(tb_len+MAT_OFFSET)*EXT_MM_M/* pitta_extrude_offset */;
      // material_tune_offset = (long)resrv1*EXT_MM_M;
      mot_ext_remain_step = material_offset;     

      SET_INPUT_PULLUP(ONE_W_CMD_PIN);
      pitta_data_state = PHY_PROCESS;
      // b_req_phy_process = true;
      b_change_done = true;
      b_rcv_confirm_need = false;
      feed_fast_cnt = 0;
    }
    else if (received_cmd == MSG_DONE ) {   
      proc_state = PROC_MSG_OK;
      if (buffer_diff>0) {
        SERIAL_ECHOLN("MSG_DONE received, buffer remain to send");
        confirmed_set_output_low();
        pitta_data_state = PREPAIR;
        b_check_wait = false;    
      }
      else {
        SERIAL_ECHOLN("MSG_DONE received, go to listen");
        SET_INPUT_PULLUP(ONE_W_CMD_PIN);
        pitta_data_state = LISTEN;
        b_rcv_confirm_need = false;        
      }
    }    
    else if (received_cmd == PTMSG_OK ) {   
      proc_state = PROC_MSG_OK;
      SERIAL_ECHOLN("PTMSG_OK");
      SET_INPUT_PULLUP(ONE_W_CMD_PIN);
      pitta_data_state = LISTEN;
      b_rcv_confirm_need = false;
    }
    else if (received_cmd == TUNE_VAL1 ) {
      proc_state = PROC_CONTINUE;
      SERIAL_ECHO("tune val 1 set:");
      SERIAL_ECHOLN(received_data);
      SERIAL_ECHOLN("move from RCV to PREPAIR for continue");
      confirmed_set_output_low();
      pitta_data_state = PREPAIR;
      tb_len = received_data;
      b_pitta_ui_force_update_req = true;
    }
    else if (received_cmd == TUNE_VAL2 ) {
      proc_state = PROC_CONTINUE;
      SERIAL_ECHO("tune val 2 set:");
      SERIAL_ECHOLN(received_data);
      SERIAL_ECHOLN("move from RCV to PREPAIR for continue");
      confirmed_set_output_low();
      pitta_data_state = PREPAIR;
      resrv1 = received_data;
    }  
    else if (received_cmd == TUNE_VAL3 ) {
      proc_state = PROC_CONTINUE;
      SERIAL_ECHO("tune val 3 set:");
      SERIAL_ECHOLN(received_data);
      SERIAL_ECHOLN("move from RCV to PREPAIR for continue");
      confirmed_set_output_low();
      pitta_data_state = PREPAIR;
      resrv2 = received_data;
    }
    else if (received_cmd == TUNE_VAL4 ) {
      proc_state = PROC_CONTINUE;
      SERIAL_ECHO("tune val 4 set:");
      SERIAL_ECHOLN(received_data);
      SERIAL_ECHOLN("move from RCV to PREPAIR for continue");
      confirmed_set_output_low();
      pitta_data_state = PREPAIR;
      resrv3 = received_data;
    }  
    else if (received_cmd == TUNE_VAL5 ) {
      proc_state = PROC_CONTINUE;
      SERIAL_ECHO("tune val 5 set:");
      SERIAL_ECHOLN(received_data);
      SERIAL_ECHOLN("move from RCV to PREPAIR for continue");
      confirmed_set_output_low();
      pitta_data_state = PREPAIR;
      resrv4 = received_data;
    }   
    else if (received_cmd == TUNE_VAL6 ) {
      proc_state = PROC_CONTINUE;
      SERIAL_ECHO("tune val 6 set:");
      SERIAL_ECHOLN(received_data);
      SERIAL_ECHOLN("move from RCV to PREPAIR for continue");
      confirmed_set_output_low();
      pitta_data_state = PREPAIR;
      ptrn_n = received_data;
    } 
    else {
      // SERIAL_ECHO("Rcv: ");
      // SERIAL_ECHOLN(received_data);
      // SERIAL_ECHOLN((unsigned)received_packet);
      #ifdef DEV_NCMD_PRINT
      wrong_cmd_cnt++;
      #endif
      SERIAL_ECHOLN("Wrong command received... *********************************************************");
      b_pitta_data_init = false;
      b_resend_req = true;
      resend_retry_cnt++;
      SERIAL_ECHOLN("move from PARSER to PREPAIR for retry request");
      confirmed_set_output_low();
      pitta_data_state = PREPAIR;      
    }
/*     else if (received_cmd == TUNE_VAL7 ) {
      proc_state = PROC_CONTINUE;
      SERIAL_ECHO("tune val 7 set:");
      SERIAL_ECHOLN(received_data);
      SERIAL_ECHOLN("move from RCV to PREPAIR for continue");
      confirmed_set_output_low();
      pitta_data_state = PREPAIR;
      pitta_val_7 = received_data;
    }     */   
  }
}

void PITTA::physical_processing() {
  static int loop_watchdog_cnt = 0;

  static int expire_resend_cnt = 0;
  extern long int extrude_init_dist;
  static int ext_acc_val = 500;
  
  if (b_ware_spreading) {

    b_ware_spreading = false;
    ware_expire_cnt = 0;
    delay(3);
    E0_STEP_WRITE(LOW);
    delay(7);
    mExtruder_dir(NOM_DIR);
    delay(10);
    ext_acc_val = 500;

  }
  if (b_pitta_ui_force_update_req) {
#ifdef MODEL_E3PRO
    lcd_status_printf_P(0, PSTR("Chg F%i & turn %i"), int(index + 1), int(change_turn_val));
#endif
#ifdef MODEL_E3V2_PRO32
#ifdef DEV_NCMD_PRINT
      ui.status_printf(0, F("PE:%i CE:%i TB_L:%i"), int(wrong_packet_cnt), int(wrong_cmd_cnt), int(tb_len));
#else
      // ui.status_printf(0, F("Chg F%i & turn %i"), int(index + 1), int(change_turn_val));
      ui.status_printf(0, F("F:%i N:%i TB_LEN:%i"), int(g_index + 1), int(change_turn_val), int(tb_len));
#endif 
#endif     
    for (int i = 0; i<20; i++) 
    {
      delay(50);
      pitta_ui_thermal_update();
    }
    b_pitta_ui_force_update_req = false;
  }

  abs_issue_trigger_remain_tick = abs_issue_trigger_remain_tick + loop_diff_tick;
  if (abs_issue_trigger_remain_tick > i_ABS_MOT_TRIGGER_CNT_DUTY)
  {
    abs_issue_trigger_remain_tick = abs_issue_trigger_remain_tick - i_ABS_MOT_TRIGGER_CNT_DUTY;
    b_state_proceed = true;
    if (jam_expire_cnt > 90000 && !b_pitta_jammed)//45000
    {
      b_pitta_jammed = true;
      mat_det_hyst_cnt = 0;
      b_jam_recovered = false;
      b_last_jam_sensor_state = b_material_detected_from_pitta;
      change_talk_expire_cnt = 0;
      jam_expire_cnt = 0;
      SERIAL_ECHOLNPGM("Pitta Jammed");
      extern int16_t temp_temp_extruder, temp_temp_bed;
      temp_temp_extruder = pitta_get_temp(0);
      pitta_set_temp(140, 0);
      SERIAL_ECHOLNPGM("safety hotend temp: ", 140);
      temp_temp_bed = pitta_get_temp_bed();
      pitta_set_temp_bed(40);
      SERIAL_ECHOLNPGM("safety bed temp: ", 40);
      diplay_update_cnt = 990;
      // SET_INPUT_PULLUP(ONE_W_CMD_PIN);
      SET_INPUT_PULLDOWN(ONE_W_CMD_PIN);
      pitta_ui_thermal_update();

    } 

    loop_watchdog_cnt++;

    if (loop_watchdog_cnt > 60)
    {
      loop_watchdog_cnt = 0;
      pitta_wtcdog_reset();
      pitta_req_manage_heater_update();
      // WRITE(E0_ENABLE_PIN, LOW);
    }

    static bool b_mot_dir = NOM_DIR;
    static int s_reset_ext_driver_cnt = 0;
    #define JAM_EXT_ADD 2000  //5000
    
    if (b_change_done)
    {
      // SERIAL_ECHOLN("PHY 1");
      b_material_detected_from_pitta = chk_material();
      if (b_material_detected_from_pitta) {
        static int l_cnt = 0;
        mat_det_hyst_cnt++;
        if (mat_det_hyst_cnt>10000) mat_det_hyst_cnt = 10000;
        pitta_wtcdog_reset();
        
        l_cnt++;
        if (l_cnt>50) {//
          l_cnt = 0;
          // ui.status_printf(0, F("hyst %i & jam %i"), int(mat_det_hyst_cnt), int(b_pitta_jammed));
          // pitta_ui_thermal_update(); 
          pitta_req_manage_heater_update();
        }
      }
      else {
        static int ll_cnt = 0;
        ll_cnt++;
        if (ll_cnt>100) {//
          ll_cnt = 0;
          // ui.status_printf(0, F("hyst_n %i & jam %i"), int(mat_det_hyst_cnt), int(b_pitta_jammed));
          // pitta_ui_thermal_update(); 
          pitta_req_manage_heater_update();
        }        
        mat_det_hyst_cnt-=2;
        if (mat_det_hyst_cnt<0) mat_det_hyst_cnt = 0;
      }

      static uint32_t previousMillis = 0, currentMillis = 0;
      currentMillis = millis();
      if ((currentMillis - previousMillis > 70)||(previousMillis-currentMillis>70)) {
        previousMillis = currentMillis;
        pitta_req_manage_heater_update();
      }

      b_mot_dir = NOM_DIR;

      if ((!b_pitta_jammed&&mat_det_hyst_cnt<900)||(b_pitta_jammed&&mat_det_hyst_cnt<4000))
      {
        s_reset_ext_driver_cnt++;
        if (s_reset_ext_driver_cnt>50000) {
          s_reset_ext_driver_cnt = 0;
          if (!b_pitta_jammed) {
            E0_ENABLE_WRITE(HIGH);
            delay(50);
            E0_ENABLE_WRITE(LOW);
            delay(50);
          }
        }
        material_offset = (long)(tb_len+MAT_OFFSET)*EXT_MM_M/* pitta_extrude_offset */;
        // material_tune_offset = (long)resrv1*EXT_MM_M;
        // mot_ext_remain_step = material_offset;  
        // material_offset = (long)tb_len*EXT_CM_M/* pitta_extrude_offset */;
        // material_tune_offset = (long)resrv1*EXT_MM_M;
        init_mot_ext_remain_step = mot_ext_remain_step = material_offset; // 46, 0 for e3v2
        if (mat_det_hyst_cnt == 0) {
          i_ABS_MOT_TRIGGER_CNT_DUTY = 200;//400
        }
        else {
          i_ABS_MOT_TRIGGER_CNT_DUTY = 700;//400
        }
        // feed_fast_cnt++;
        // if (feed_fast_cnt<8000) {
        //   i_ABS_MOT_TRIGGER_CNT_DUTY = 200;//400
        // }
        // else {
        //   i_ABS_MOT_TRIGGER_CNT_DUTY = 700;//400
        //   feed_fast_cnt = 8000;
        // }
      }
      else
      {
// #ifdef MODEL_E3PRO
//         lcd_status_printf_P(0, PSTR("Chg F%i & turn %i"), int(index + 1), int(change_turn_val));
// #endif
// #ifdef MODEL_E3V2_PRO32
//         ui.status_printf(0, F("stp e%i & turn %i"), int(jam_expire_cnt), int(change_turn_val));
// #endif           
        // delay(10);
        // mExtruder_dir(b_mot_dir);
        // delay(10);
        int expire_cnt = 0;
        b_y_spread = false;
        s_reset_ext_driver_cnt = 0;
        
        int y_spread_cur_pos = 0;
        bool b_y_spread_finish_req = false;
        int lcd_loop_update_cnt = 0;
        if (b_pitta_jammed) {
          SET_INPUT_PULLUP(ONE_W_CMD_PIN);
          b_pitta_jammed = false;
          b_jam_recovered = true;
          pitta_set_temp(temp_temp_extruder, 0);
          SERIAL_ECHOLNPGM("restore extruder temp: ", temp_temp_extruder);
          pitta_set_temp_bed((temp_temp_bed));
          SERIAL_ECHOLNPGM("restore bed temp: ", temp_temp_bed);
          SERIAL_ECHOLNPGM("jam resolved"); 
          // material_offset = (long)tb_len*EXT_CM_M/* pitta_extrude_offset */;
          // material_tune_offset = (long)resrv1*EXT_MM_M;
          material_offset = (long)(tb_len+MAT_OFFSET)*EXT_MM_M /* pitta_extrude_offset */;
          init_mot_ext_remain_step = mot_ext_remain_step = material_offset + JAM_EXT_ADD;     
          // pitta_ui_thermal_update();   
          while(pitta_get_celsius(0)<185) {
            delay(50);
            pitta_wtcdog_reset();
            pitta_ui_thermal_update(); 
          }
          
        }
        pitta_ui_thermal_update(); 
        // pitta_ui_update();
        delay(1);
        while (mot_ext_remain_step > 0 || b_y_spread)
        {
          lcd_loop_update_cnt++;
          if (lcd_loop_update_cnt>2000) {
            lcd_loop_update_cnt = 0;
          }
          currentMillis = millis();
          if ((currentMillis - previousMillis > 20)||(previousMillis-currentMillis>20)) {
            previousMillis = currentMillis;
            pitta_req_manage_heater_update();
          }

          if (mot_ext_remain_step == 300*MUL_V)  // 4500
          {
             if (b_use_spread_tower) {
              b_y_spread = true;
              y_spread_cur_pos = y_shift_pos;
             }
             else {

             }
          }

          if (b_y_spread) {
            if (/* !b_spread_dir &&*/b_y_spread_finish_req) {
              if (y_spread_cur_pos>0) {
                Y_STEP_WRITE(HIGH);
                y_spread_cur_pos--;
              }

              if (y_spread_cur_pos<1) {
                delayMicroseconds(1300/WAIT_MUL_V);
                b_y_spread = false;
                Y_DIR_WRITE(INVERT_Y_DIR ? !b_y_dir : b_y_dir); 
                delayMicroseconds(500/WAIT_MUL_V);
                Y_STEP_WRITE(LOW);
                delayMicroseconds(1800/WAIT_MUL_V); 
                b_y_spread_finish_req = false;
              }
            }
          }
          E0_STEP_WRITE(HIGH);
          if (b_jam_recovered) {
            delayMicroseconds(400/WAIT_MUL_V); //600
          }

          mot_ext_remain_step-=1;
    
#define EXT_SLOW_SPD 65//60//100//70
          if (mot_ext_remain_step > init_mot_ext_remain_step - 400*MUL_V)
          {
            if (mot_ext_remain_step == init_mot_ext_remain_step-100*MUL_V) {
              temp_temp_extruder = pitta_get_temp(0);
              pitta_ui_thermal_update(); 
            }

            delayMicroseconds(210/WAIT_MUL_V);//120
            E0_STEP_WRITE(LOW);
            delayMicroseconds(210/WAIT_MUL_V);//120
          }
          else if (mot_ext_remain_step > init_mot_ext_remain_step - 2000*MUL_V)
          {

            delayMicroseconds(90/WAIT_MUL_V);//120
            E0_STEP_WRITE(LOW);
            delayMicroseconds(90/WAIT_MUL_V);//120
          }          
          else if (mot_ext_remain_step > 8000*MUL_V)//7000
          {
            {
              delayMicroseconds((EXT_SLOW_SPD /* + pitta_extrude_return_spd */)/MUL_V);  // 35
              E0_STEP_WRITE(LOW);
              delayMicroseconds((EXT_SLOW_SPD /* + pitta_extrude_return_spd */)/MUL_V);
            }
          }
          else if (mot_ext_remain_step > 1400)
          {
            delayMicroseconds(200/WAIT_MUL_V);//400
            E0_STEP_WRITE(LOW);
            Y_STEP_WRITE(LOW);
            delayMicroseconds(200/WAIT_MUL_V);
          }    

          /// from 1500        
          else if (mot_ext_remain_step > 1000)
          {
            delayMicroseconds(350/WAIT_MUL_V);//600
            E0_STEP_WRITE(LOW);
            Y_STEP_WRITE(LOW);
            delayMicroseconds(350/WAIT_MUL_V);
          }          
          else if (mot_ext_remain_step > 900)
          {
            delayMicroseconds(600/WAIT_MUL_V);//900
            E0_STEP_WRITE(LOW);
            Y_STEP_WRITE(LOW);
            delayMicroseconds(600/WAIT_MUL_V);
            expire_cnt++;
          }

          /// from 500
          else if (mot_ext_remain_step > 150)
          {
            if (mot_ext_remain_step>300) {
              for (int i = 0;i<0;i++) {  //resrv3:0
                ext_flat(INV_DIR, 150, 50);
                ext_flat(INV_DIR, 50, 1800);
                ext_flat(INV_DIR, 150, 50);


                ext_flat(NOM_DIR, 150, 50);
                ext_flat(NOM_DIR, 50, 1820);
                ext_flat(NOM_DIR, 150, 50);
                pitta_wtcdog_reset();
                pitta_req_manage_heater_update();
              }
              mot_ext_remain_step = 200;
            }

            delayMicroseconds(600/WAIT_MUL_V);//1000
            E0_STEP_WRITE(LOW);
            Y_STEP_WRITE(LOW);
            delayMicroseconds(600/WAIT_MUL_V);
            expire_cnt+=5;
          }
          else
          {
            if (mot_ext_remain_step == 50*MUL_V) {//1200
              if (b_y_spread) {
                b_y_spread_finish_req = true;
                Y_DIR_WRITE(INVERT_Y_DIR ? !SPREAD_RES_DIR : SPREAD_RES_DIR);
              }
              pitta_set_temp(temp_temp_extruder, 0);
              pitta_ui_thermal_update(); 
            } 
            if (mot_ext_remain_step<0) {
              mot_ext_remain_step = 0;
              delayMicroseconds(450/WAIT_MUL_V);//450
              E0_STEP_WRITE(LOW);
              delayMicroseconds(450/WAIT_MUL_V);//450    
              expire_cnt++;          
            }
            else {
              {
                delayMicroseconds(1200/WAIT_MUL_V);//1200
                E0_STEP_WRITE(LOW);
                delayMicroseconds(1200/WAIT_MUL_V);
                expire_cnt+=6;
              }
            }
            i_ABS_MOT_TRIGGER_CNT_DUTY = 2000;
          }
          if (b_y_spread) {
            delayMicroseconds(120/WAIT_MUL_V);
            Y_STEP_WRITE(LOW);
            delayMicroseconds(120/WAIT_MUL_V);
          }
        }
        
        {
          
          int r_turn = 7;
          if (resrv4 == 10)
          {
          delay(5);
          for (int i = 0; i<r_turn; i++ ) {
            ext_flat(INV_DIR, TURN_PASS, 15);
            ext_flat(INV_DIR, SLOW_PASS, 10);
            ext_flat(INV_DIR, 110, 20);
            ext_flat(INV_DIR, 90, 20);
            ext_flat(INV_DIR, 80, 20);
            ext_flat(INV_DIR, 70, 15);
            ext_flat(INV_DIR, FAST_PASS, 100*3);// 110
            ext_flat(INV_DIR, 70, 15);
            ext_flat(INV_DIR, 80, 20);
            ext_flat(INV_DIR, 90, 20);
            ext_flat(INV_DIR, 110, 20);
            ext_flat(INV_DIR, SLOW_PASS, 15);
            ext_flat(INV_DIR, TURN_PASS, 10);

            ext_flat(NOM_DIR, TURN_PASS, 10);
            ext_flat(NOM_DIR, SLOW_PASS, 15);
            ext_flat(NOM_DIR, 110, 20);
            ext_flat(NOM_DIR, 90, 20);
            ext_flat(NOM_DIR, 80, 20);
            ext_flat(NOM_DIR, 70, 15);
            ext_flat(NOM_DIR, FAST_PASS, 100*3);// 110
            ext_flat(NOM_DIR, 70, 15);
            ext_flat(NOM_DIR, 80, 20);
            ext_flat(NOM_DIR, 90, 20);
            ext_flat(NOM_DIR, 110, 20);
            ext_flat(NOM_DIR, SLOW_PASS, 10);
            ext_flat(NOM_DIR, TURN_PASS, 15);
          } 
          delay(5);
          }
          // material_offset = (long)tb_len*EXT_CM_M/* pitta_extrude_offset */;
          // material_tune_offset = (long)resrv1*EXT_MM_M;
          material_offset = (long)(tb_len+MAT_OFFSET)*EXT_MM_M/* pitta_extrude_offset */;
          mot_ext_remain_step = material_offset;
          SET_INPUT_PULLUP(ONE_W_CMD_PIN);
          pitta_sel_req = false;
          SERIAL_ECHOLNPGM("extrude ready: ", mot_ext_remain_step);
          SERIAL_ECHOLNPGM("expire_retry_cnt : ", expire_resend_cnt);
          b_jam_recovered = false;
          i_ABS_MOT_TRIGGER_CNT_DUTY = 2000;//1500

          b_change_done = false;
          mat_det_hyst_cnt = 0;
          pitta_enable_e0();
          b_pitta_lock = false;
          ext_load_damp = EXT_SLOW_LOAD;
          b_slave_as_invalid_sensor = true;
          pitta_set_temp(pitta_get_temp(0), 0);  

          SET_INPUT_PULLDOWN(ONE_W_CMD_PIN);
          pitta_data_state = LISTEN; 
        }
      }
      static bool b_step = false;
      if (mot_ext_remain_step > 0 && b_pitta_lock && !b_pitta_jammed)
      {
        b_step = !b_step;
        if (cur_ext_dir != b_mot_dir) {
          delay(8);
          E0_STEP_WRITE(LOW);
          delay(10);
          mExtruder_dir(b_mot_dir);
          delay(10);
        } 
        if (ext_acc_val>0) {
          ext_acc_val--;
          if (b_step)
          {
            delayMicroseconds(400);
            E0_STEP_WRITE(HIGH);
            jam_expire_cnt++;
          }
          else
          {
            delayMicroseconds(400);
            E0_STEP_WRITE(LOW);
          }
        }
        else {
          if (b_step)
          {
            delayMicroseconds(150);
            E0_STEP_WRITE(HIGH);
            jam_expire_cnt++;
            
          }
          else
          {
            delayMicroseconds(150);
            E0_STEP_WRITE(LOW);
          }
        }
      }
    }
  }
}

void PITTA::data_prepair() {
  static long temp_var;
  b_state_proceed = true;
  if (!b_resend_req)
  {
    if (proc_state == PROC_CONTINUE) {
      temp_var = COM_CONTINUE;
      b_rcv_confirm_need = true;
      resend_var = temp_var;
      SERIAL_ECHOLNPGM("PROC_CONTINUE: ", temp_var);
    }
    else {
      temp_var = get_data_from_buffer();
      temp_var = temp_var&0xffffff;
      b_rcv_confirm_need = true;
      resend_var = temp_var;
      SERIAL_ECHOLNPGM("NORM DATA SEND: ", temp_var);
    }
  }
  else
  {
    if (resend_retry_cnt>2) {
      resend_retry_cnt = 0;
      SERIAL_ECHOLN("RETRY WAIT 2 SEC");
      for (int i = 0; i<20; i++) {
        delay(100);
        pitta_wtcdog_reset();
        pitta_req_manage_heater_update();
      }
    }
    temp_var = resend_var;
    resend_var = temp_var;
    SERIAL_ECHOLNPGM("SEND FAIL, RETRY: ", temp_var);
    b_resend_req = false;
  }
  SERIAL_ECHOLNPGM("pitta.data: ", temp_var);
  send_packet = 0xffffff & temp_var;
  long chk_high8 = 0;
  for (int i = 0;i<24; i++) {
      chk_high8 = chk_high8 + (((0xffffff & send_packet)>>i)&0x1)*(i+0)*2;
  }
  chk_high8 = chk_high8 & 0xff;
  send_packet = send_packet|(chk_high8<<24);
  b_update_M_send_data = true;
  b_start_send_bit = true;
  b_pitta_sending = true;
  send_bit_shift = 0;
  io_chk_n_set_as_output(ONE_W_CMD_PIN);
  WRITE(ONE_W_CMD_PIN, LOW);  
  safe_delay(300);//800//300 
  pitta_data_state = SENDING; 
  pitta_wtcdog_reset();
}

void PITTA::sending() {
  abs_comm_send_remain_tick = abs_comm_send_remain_tick + loop_diff_tick;
  b_state_proceed = true;
  if (abs_comm_send_remain_tick > ABS_COMM_SEND_CNT_DUTY)
  {
    abs_comm_send_remain_tick = abs_comm_send_remain_tick - ABS_COMM_SEND_CNT_DUTY;
    if (b_start_send_bit)
    {
      b_send_bit = (bool)((send_packet >> send_bit_shift) & 1);
      if (b_send_bit)
      {
        send_hi_turn = 50; //45//80
        send_lo_turn = 5;  //3
      }
      else
      {
        send_hi_turn = 10;//4;//3
        send_lo_turn = 5; //3
      }
      send_bit_shift++;
      b_start_send_bit = false;
      if (send_bit_shift == 1) {
        SERIAL_ECHOLN("<Start--Send>");
      }
      if (send_bit_shift == 33)
      {
        b_pitta_sending = false;
        send_bit_shift = 0;
        b_wire_sent_byte = true;
        WRITE(ONE_W_CMD_PIN, LOW);
        SERIAL_ECHOLNPGM("M Snd: ", 0x00ff & send_packet);
        b_update_M_send_data = false;
        sent_packet = send_packet;
        b_pitta_loop_lock = false;
        SET_INPUT_PULLUP(ONE_W_CMD_PIN);
        pitta_data_state = LISTEN; 
        rcv_packet_expire = 0;
        SERIAL_ECHOLN("<End-----Send>");
        pitta_req_manage_heater_update();         
      }
    }
    else if (send_hi_turn + send_lo_turn > 0)
    {
      if (send_hi_turn > 0)
      {
        WRITE(ONE_W_CMD_PIN, HIGH);
        send_hi_turn--;
      }
      else
      {
        WRITE(ONE_W_CMD_PIN, LOW);
        send_lo_turn--;
      }
    }
    else
    {
      b_start_send_bit = true;
      b_pitta_sending = true;
      pitta_wtcdog_reset();
      pitta_req_manage_heater_update();
    }
  }  
}

void PITTA::state_processing() {
  b_state_proceed = true;
  if (b_ok_ready_to_say) {
    if (buffer_diff>0||b_resend_req) {
      confirmed_set_output_low();
      pitta_data_state = PREPAIR;
    }
    else {
      SET_INPUT_PULLUP(ONE_W_CMD_PIN);
      pitta_data_state = LISTEN;        
    }
  }
  else {
    SET_INPUT_PULLUP(ONE_W_CMD_PIN);
    pitta_data_state = LISTEN;  
  }
  pitta_req_manage_heater_update();
}

#endif // HAS_PITTA_MMU