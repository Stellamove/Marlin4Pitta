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
#pragma once

#include "../../inc/MarlinConfig.h"

#define PITTA_RX_SIZE 4
#define PITTA_TX_SIZE 4

extern int16_t pitta_extrude_offset, pitta_ext_m_offset[];
extern int16_t pitta_retract_full_offset;
extern int16_t pitta_extrude_turn_val, pitta_extrude_turn_B_val,pitta_extrude_turn_C_val, pitta_extrude_1st_retract_dist, pitta_extrude_lowest_extrude_dist, pitta_extrude_return_spd;

struct E_Step;

class PITTA {
public:
  PITTA();

  static void init();
  static void pitta_stop();
  static void reset();
  static void pitta_loop();
  static void pitta_act();
  static uint8_t get_current_tool();
  static void e0_state_set(const bool dir, const int spd);
  static bool unload();
  static void fila_change(const uint8_t index);
  static void fila_select(const uint8_t index);
  static void fila_retract(const uint8_t index);
  static void fila_extrude(const uint8_t index);
  static void send_force_cmd(const int cmd);
  static void enable(const bool b_enable);
  static void pitta_state_proceed();
  static uint8_t pitta_state;
  static bool pitta_stop_req, pitta_enabled, b_retract_ready, b_pitta_just_enabled;
  static bool pitta_sel_req, b_pitta_loop_lock, b_e0_dir, b_init_pitta
  ,b_slave_as_sensor, b_slave_as_invalid_sensor, b_init_pitta_initialized, b_material_detected_from_pitta,b_pitta_lock;
  static int send_M_byte_remain_num, e0_spd;
  static int change_turn_val;

private:
    static bool rx_str_P(const char* str);
    static void tx_str_P(const char* str);
    static void tx_printf_P(const char* format, const int argument);
    static void clear_rx_buffer();

    static bool pitta_start();
    static bool pitta_ok();

    static void command(const uint8_t cmd);
    static bool get_response();
    static void manage_response(const bool move_axes, const bool turn_off_nozzle);

    static void execute_extruder_sequence(const E_Step * sequence, int steps);

    static void filament_runout();
    static void on_listen();
    static void on_receiving();
    static void data_prepair();
    static void sending();
    static void state_processing();
    static void parsing();
    static void physical_processing();

    static bool pitta_ready, pitta_print_saved;
    static unsigned int prev_main_sys_tick, main_sys_tick;
    static unsigned int loop_diff_tick;
    static unsigned int abs_comm_M_remain_tick;
    static unsigned int abs_comm_receive_remain_tick;
    static unsigned int abs_comm_send_remain_tick;
    static unsigned int abs_issue_trigger_remain_tick;
    static int must_ready_hear_cnt;
    static bool b_ok_ready_to_say;
    static bool b_on_hearing, b_on_saying;

    static long receive_bit_shift;
    static long receive_byte;
    static bool b_send_bit;
    static int send_hi_turn;
    static int send_lo_turn;
    // static unsigned int send_byte, sent_byte;
    // static unsigned int send_low8, send_high8;
    static long send_packet, sent_packet;
    static unsigned int send_cmd, send_data, send_chk;    
    static long send_bit_shift;
    static bool b_start_send_bit;

    static unsigned int receive_tick_hi_cnt;
    static unsigned int receive_tick_lo_cnt;
    // static unsigned int received_byte;
    static long received_packet;
    static unsigned int received_cmd;
    static int received_data;

    static bool b_byte_receive_done;

    static int pitta_log_cnt;

    static uint8_t cmd, last_cmd, extruder;
    // static uint8_t pitta_state;
    static volatile int8_t fila_sens;
    static volatile bool fila_runout_valid;
    static int16_t version;

    static millis_t prev_request, prev_FS_request;
    static char rx_buffer[PITTA_RX_SIZE], tx_buffer[PITTA_TX_SIZE];

    static inline void set_runout_valid(const bool valid) {
      fila_runout_valid = valid;
    }
};

extern PITTA pitta;