/*
Copyright 2011 Jun Wako <wakojun@gmail.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * scan matrix
 */
#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>
#include "print.h"
#include "util.h"
#include "debug.h"
#include "host.h"
#include "led.h"
#include "m0110.h"
#include "matrix.h"


#define CAPS        0x39
#define CAPS_UP     0xB9 //(CAPS | 0x80)
#define SHIFT       0x38
#define SHIFT_UP    0xB8
#define ROW(key)    ((key)>>3&0x0F)
#define COL(key)    ((key)&0x07)


static bool is_modified = false;

// matrix state buffer(1:on, 0:off)
static uint8_t *matrix;
static uint8_t _matrix0[MATRIX_ROWS];

#ifdef MATRIX_HAS_GHOST
static bool matrix_has_ghost_in_row(uint8_t row);
#endif
static void register_key(uint8_t key);


inline
uint8_t matrix_rows(void)
{
    return MATRIX_ROWS;
}

inline
uint8_t matrix_cols(void)
{
    return MATRIX_COLS;
}

void matrix_init(void)
{
    print_enable = true;
    debug_enable = true;
    debug_matrix = false;
    debug_keyboard = false;
    debug_mouse = false;
    print("debug enabled.\n");

    m0110_init();
    // initialize matrix state: all keys off
    for (uint8_t i=0; i < MATRIX_ROWS; i++) _matrix0[i] = 0x00;
    matrix = _matrix0;
    return;
}

/*
 matrix_scan
 Basic function to report any key events.
 Requests a report from the keyboard, tries to resolve the source of any 'Shift' plus keypad/arrow
 events and registers keyboard events to the matrix.
 */
uint8_t matrix_scan(void)
{
    uint8_t key, keyaux;

    is_modified = false;
    key = m0110_recv_key();

#ifdef MATRIX_HAS_LOCKING_CAPS
    // Send Caps key up event
    if (matrix_is_on(ROW(CAPS), COL(CAPS))) {
        is_modified = true;
        register_key(CAPS_UP);
    }
#endif
    if (key == M0110_NULL)      // If the key event is null, return with no event
    {
        return 0;
    }
    else if (key == M0110_SHIFT_DN || key == M0110_SHIFT_UP)    // If there is a 'Shift' event, request
    {                                                            // a new instant response from the keyboard,
                                                                 // in case it's a 'calc' key sequence.
      keyaux = m0110_inst_key();

      is_modified = true;

      if (keyaux == M0110_NULL)   // If the received event is null, then it's a plain 'shift' event
      {
          register_key(key);
      }
      else if (!(keyaux&0x60))    // If the received event is a plain key (not arrow or keypad) then
      {                            // register both shift event and key event
          register_key(keyaux);
          register_key(key);
      }
      else if ((key & 0x80) != (keyaux & 0x80))    // If the shift event is 'release' but the second event is
      {                                             // press, then it's a user-generated event and not a calc key
          register_key(keyaux&0xDF);
          register_key(key);
      }
      else        // If the received event is not null and not plain key, things get complicated
      {
       // We know that the received key is either keypad, arrow or calc; moreover, we assume that any
       // simultaneous events will deal with shift plus arrow key simultaneous releases. It is not possible
       // to distinguish a simultaneous shift plus arrow key press from a 'virtual' shift plus arrow key
       // press event, generated by the 'calc' keys.

       // If the received key release is a 'calc' key and not already registered as pressed, but the arrow key
       // with the same scancode is registered as pressed, then we understand that we've got a simultaneous
       // shift-arrow release; then readjust the received key code to 'arrow' (0x40) instead of 'calc' (0x60)
       // (i.e. subtract 0x20)
          if (keyaux&0xE0 == 0xE0)        // If event is release; quick check before matrix check
            if (!matrix_is_on(ROW(keyaux), COL(keyaux)) && \
                matrix_is_on(ROW(keyaux&0xDF), COL(keyaux&0xDF)))
                keyaux &= 0xDF;

       // Register the second key event
          register_key(keyaux);

       // If the received key is not a 'calc' key, in which case the 'shift' event is not a 'virtual' one,
       // then register the 'shift' key event
          if ((keyaux&0x60) != 0x60)
            register_key(key);
      }
    }
    else              // In any other case, move on for the final check
    {
#ifdef MATRIX_HAS_LOCKING_CAPS    
        if (host_keyboard_leds() & (1<<USB_LED_CAPS_LOCK)) {
            // CAPS LOCK on:
            // Ignore LockingCaps key down event
            if (key == CAPS) return 0;
            // Convert LockingCaps key up event into down event
            if (key == CAPS_UP) key = CAPS;
        } else {
            // CAPS LOCK off:
            // Ignore LockingCaps key up event
            if (key == CAPS_UP) return 0;
        }
#endif
        is_modified = true;

     // If a calc key is pressed while the shift key is already pressed by the user, the keyboard transmits
     // a full 3-byte sequence for the keypress event but omits the shift-release event (in order not to
     // disrupt the user-pressed shift state) and returns a 2-byte release event. Therefore, we have to
     // 'convert' the key release event from wrongly interpreted as arrow release into a calc key release.
        if ((key&0xE0) == 0xC0)       // If event is release and key is keypad; fast check before matrix check
            if (!matrix_is_on(ROW(key), COL(key)) && \
                  matrix_is_on(ROW(key | 0x20), COL(key | 0x20)))
                 key |= 0x20;

          register_key(key);
    }

    if (debug_enable) {
        print("key: "); phex(key); print("\n");
    }
    return 1;
}

bool matrix_is_modified(void)
{
    return is_modified;
}

inline
bool matrix_has_ghost(void)
{
#ifdef MATRIX_HAS_GHOST
    for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
        if (matrix_has_ghost_in_row(i))
            return true;
    }
#endif
    return false;
}

inline
bool matrix_is_on(uint8_t row, uint8_t col)
{
    return (matrix[row] & (1<<col));
}

inline
uint8_t matrix_get_row(uint8_t row)
{
    return matrix[row];
}

void matrix_print(void)
{
    print("\nr/c 01234567\n");
    for (uint8_t row = 0; row < matrix_rows(); row++) {
        phex(row); print(": ");
        pbin_reverse(matrix_get_row(row));
        print("\n");
    }
}

uint8_t matrix_key_count(void)
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
        count += bitpop(matrix[i]);
    }
    return count;
}

#ifdef MATRIX_HAS_GHOST
inline
static bool matrix_has_ghost_in_row(uint8_t row)
{
    // no ghost exists in case less than 2 keys on
    if (((matrix[row] - 1) & matrix[row]) == 0)
        return false;

    // ghost exists in case same state as other row
    for (uint8_t i=0; i < MATRIX_ROWS; i++) {
        if (i != row && (matrix[i] & matrix[row]) == matrix[row])
            return true;
    }
    return false;
}
#endif

inline
static void register_key(uint8_t key)
{
    if (key&0x80) {
        matrix[ROW(key)] &= ~(1<<COL(key));
    } else {
        matrix[ROW(key)] |=  (1<<COL(key));
    }
}
