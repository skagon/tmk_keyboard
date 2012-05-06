/*
Copyright 2011 Jun WAKO <wakojun@gmail.com>

This software is licensed with a Modified BSD License.
All of this is supposed to be Free Software, Open Source, DFSG-free,
GPL-compatible, and OK to use in both free and proprietary applications.
Additions and corrections to this file are welcome.


Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in
  the documentation and/or other materials provided with the
  distribution.

* Neither the name of the copyright holders nor the names of
  contributors may be used to endorse or promote products derived
  from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "m0110.h"
#include "debug.h"


static inline void clock_lo(void);
static inline void clock_hi(void);
static inline bool clock_in(void);
static inline void data_lo(void);
static inline void data_hi(void);
static inline bool data_in(void);
static inline uint16_t wait_clock_lo(uint16_t us);
static inline uint16_t wait_clock_hi(uint16_t us);
static inline uint16_t wait_data_lo(uint16_t us);
static inline uint16_t wait_data_hi(uint16_t us);
static inline void idle(void);
static inline void request(void);


/*
Primitive M0110 Library for AVR
==============================


Signaling
---------
CLOCK is always from KEYBOARD. DATA are sent with MSB first.

1) IDLE: both line is high.
    CLOCK ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    DATA  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

2) KEYBOARD->HOST: HOST reads bit on rising edge.
    CLOCK ~~~~~~~~~~~~|__|~~~|__|~~~|__|~~~|__|~~~|__|~~~|__|~~~|__|~~~|__|~~~~~~~~~~~
    DATA  ~~~~~~~~~~~~X777777X666666X555555X444444X333333X222222X111111X000000X~~~~~~~
                      <--> 160us(clock low)
                         <---> 180us(clock high)

3) HOST->KEYBOARD: HOST asserts bit on falling edge.
    CLOCK ~~~~~~~~~~~~|__|~~~|__|~~~|__|~~~|__|~~~|__|~~~|__|~~~|__|~~~|__|~~~~~~~~~~~
    DATA  ~~~~~~|_____X777777X666666X555555X444444X333333X222222X111111X000000X~~~~~~~
                <----> 840us(request to send by host)                     <-> 80us(hold DATA)
                      <--> 180us(clock low)
                         <---> 220us(clock high)


Protocol
--------
COMMAND:
    Inquiry     0x10    get key event
    Instant     0x12    get key event
    Model       0x14    get model number(M0110 responds with 0x09)
                        bit 7   1 if another device connected(used when keypad exists?)
                        bit4-6  next device model number
                        bit1-3  keyboard model number
                        bit 0   always 1
    Test        0x16    test(ACK:0x7D/NAK:0x77)

KEY EVENT:
    bit 7       key state(0:press 1:release)
    bit 6-1     scan code
    bit 0       always 1
    To get scan code,  use ((bits&(1<<7)) | ((bits&7F))>>1).

SCAN CODE:
    M0110
    ,---------------------------------------------------------.
    |  `|  1|  2|  3|  4|  5|  6|  7|  8|  9|  0|  -|  =|Backs|
    |---------------------------------------------------------|
    |Tab  |  Q|  W|  E|  R|  T|  Y|  U|  I|  O|  P|  [|  ]|  \|
    |---------------------------------------------------------|
    |CapsLo|  A|  S|  D|  F|  G|  H|  J|  K|  L|  ;|  '|Return|
    |---------------------------------------------------------|
    |Shift   |  Z|  X|  C|  V|  B|  N|  M|  ,|  ,|  /|        |
    `---------------------------------------------------------'
         |Opt|Mac |         Space               |Enter|Opt|
         `------------------------------------------------'
    ,---------------------------------------------------------.
    | 32| 12| 13| 14| 15| 17| 16| 1A| 1C| 19| 1D| 1B| 18|   33|
    |---------------------------------------------------------|
    |   30| 0C| 0D| 0E| 0F| 10| 11| 20| 22| 1F| 23| 21| 1E| 2A|
    |---------------------------------------------------------|
    |    39| 00| 01| 02| 03| 05| 04| 26| 28| 25| 29| 27|    24|
    |---------------------------------------------------------|
    |      38| 06| 07| 08| 09| 0B| 2D| 2E| 2B| 2F| 2C|      38|
    `---------------------------------------------------------'
         | 3A|  37|             31              |   34| 3A|
         `------------------------------------------------'


    ,---------------------------------------------------------. ,---------------.
    |  `|  1|  2|  3|  4|  5|  6|  7|  8|  9|  0|  -|  =|Bcksp| |Clr|  =|  /|  *|
    |---------------------------------------------------------| |---------------|
    |Tab  |  Q|  W|  E|  R|  T|  Y|  U|  I|  O|  P|  [|  ]|   | |  7|  8|  9|  -|
    |-----------------------------------------------------'   | |---------------|
    |CapsLo|  A|  S|  D|  F|  G|  H|  J|  K|  L|  ;|  '|Return| |  4|  5|  6|  +|
    |---------------------------------------------------------| |---------------|
    |Shift   |  Z|  X|  C|  V|  B|  N|  M|  ,|  ,|  /|Shft|Up | |  1|  2|  3|   |
    |---------------------------------------------------------' |-----------|Ent|
    |Optio|Mac    |           Space           |  \|Lft|Rgt|Dn | |      0|  .|   |
    `---------------------------------------------------------' `---------------'
    ,---------------------------------------------------------. ,---------------.
    | 32| 12| 13| 14| 15| 17| 16| 1A| 1C| 19| 1D| 1B| 18|   33| | 07| 08| 0D| 02|
    |---------------------------------------------------------| |---------------|
    |   30| 0C| 0D| 0E| 0F| 10| 11| 20| 22| 1F| 23| 21| 1E|   | | 19| 1B| 1C| 1E|
    |-----------------------------------------------------'   | |---------------|
    |    39| 00| 01| 02| 03| 05| 04| 26| 28| 25| 29| 27|    24| | 16| 17| 18| 06|
    |---------------------------------------------------------| |---------------|
    |      38| 06| 07| 08| 09| 0B| 2D| 2E| 2B| 2F| 2C|  38| 0D| | 13| 14| 15|   |
    |---------------------------------------------------------' |-----------| 0C|
    |   3A|     37|            31             | 2A| 06| 02| 08| |     12| 01|   |
    `---------------------------------------------------------' `---------------'
Note: On the M0110A, the numpad keys and the arrow keys are preceded by 0x79 (scan code 3C)
      Moreover, the numpad keys =, /, * and + are preceded by shift-down (0x71 or 38) on press
      and shift-up (0xF1) on release. So, the data transferred by nupmad 5 is "79 2F"
      whereas for numpad + it's "71 79 0D".



References
----------
Protocol:
    http://www.mac.linux-m68k.org/devel/plushw.php
Connector:
    http://www.kbdbabel.org/conn/kbd_connector_macplus.png
Signaling:
    http://www.kbdbabel.org/signaling/kbd_signaling_mac.png
    http://typematic.blog.shinobi.jp/Entry/14/
Scan Codes:
    http://m0115.web.fc2.com/m0110.jpg
    http://m0115.web.fc2.com/m0110a.jpg
*/

/*
 WAIT_US
 Local macro, calls one of functions wait_data/clock_hi/lo and returns error if it returns false.
 Wait time in microseconds
 */
#define WAIT_US(stat, us, err) do { \
    if (!wait_##stat(us)) { \
        m0110_error = err; \
        goto ERROR; \
    } \
} while (0)

#define WAIT WAIT_US

/*
 WAIT_MS
 Local macro, calls one of functions wait_data/clock_hi/lo and returns error if it returns false.
 Wait time in milliseconds
 */
#define WAIT_MS(stat, ms, err) do { \
    uint16_t _ms = ms; \
    while (_ms) { \
        if (wait_##stat(1000)) { \
            break; \
        } \
        _ms--; \
    } \
    if (_ms == 0) { \
        m0110_error = err; \
        goto ERROR; \
    } \
} while (0)

uint8_t m0110_error = 0;

/*
 m0110_init
 Basic initialisation of the keyboard.
 */
void m0110_init(void)
{
    uint8_t data;
    idle();
    _delay_ms(1000);

    m0110_send(M0110_MODEL);
    data = m0110_recv();
    print("m0110_init model: "); phex(data); print("\n");

// TO DO: m0110 returns model 0x09 while m0110A returns 0x0B
// some code handling the model numbers would be nice
/*    m0110 : 0x09  00001001 : model number 4 (100)
      m0110A: 0x0B  00001011 : model number 5 (101)*/

    m0110_send(M0110_TEST);
    data = m0110_recv();
    print("m0110_init test: "); phex(data); print("\n");
}

/*
 m0110_send
 Basic function for transmitting one byte to the keyboard.
 */
uint8_t m0110_send(uint8_t data)
{
    m0110_error = 0;

    request();
    WAIT_MS(clock_lo, 250, 1);
    for (uint8_t bit = 0x80; bit; bit >>= 1) {
        WAIT(clock_lo, 250, 3);
        //_delay_us(15);
        if (data&bit) {
            data_hi();
        } else {
            data_lo();
        }
        WAIT(clock_hi, 200, 4);
    }
    _delay_us(100); // hold last bit for 80us
    idle();
    return 1;
ERROR:
    print("m0110_send err: "); phex(m0110_error); print("\n");
    _delay_ms(500);
    idle();
    return 0;
}

/*
 m0110_recv
 Basic function for receiving one byte from the keyboard.
 */
uint8_t m0110_recv(void)
{
    uint8_t data = 0;
    m0110_error = 0;

    WAIT_MS(clock_lo, 250, 1);
    for (uint8_t i = 0; i < 8; i++) {
        data <<= 1;
        WAIT(clock_lo, 200, 2);
        WAIT(clock_hi, 200, 3);
        if (data_in()) {
            data |= 1;
        }
    }
    idle();
    return data;
ERROR:
    print("m0110_recv err: "); phex(m0110_error); print("\n");
    _delay_ms(500);
    idle();
    return 0xFF;
}

/*
 m0110_inst_recv
 Instant receive function. Transmits the "instant" command to the keyboard and then receives
 one byte in reply.
 The instant command instructs the keyboard to report immediately (read description of normal
 key receive for more information).
 */
uint8_t m0110_inst_recv(void)
{
    m0110_send(M0110_INSTANT);

    return m0110_recv();
}

/*
 m0110_recv_key
 Receive key function. Transmits an "inquiry" command to the keyboard and receives one byte
 in reply.
 IMPORTANT: after an "inquiry", the keyboard could take up to 500msec to respond. The Apple
 specification allows the M0110 keyboard up to 250msec to reply, whereas the Macintosh will
 issue a reinitialisation command (0x16) after not receiving a reply for 500msec. It is unclear
 how much time the combination of M0110 keyboard plus M0120 keypad take to respond, however the
 500msec limit remains.
 The "instant" command replies instantly, without any such delay.
 */
uint8_t m0110_recv_key(void)
{
    uint8_t key;

    m0110_send(M0110_INQUIRY);

    key = m0110_recv();

    if (key == M0110_PAD_CODE)    // If the reply is the keypad prefix, issue an 'immediate' command
    {                             // to receive the actual key scancode and add 0x40 to the key code.
      key = m0110_inst_recv();
      return ((key & 0x80) | ((key & 0x7F)>>1)) | M0110_PAD_ADD;
    }
    else if (key == 0xFF || key == M0110_NULL)    // if the reply is null, return null
        return M0110_NULL;
    else                          // and if the reply is a normal key, return a normal key code
        return (key & 0x80) | ((key & 0x7F)>>1);
}

/*
 m0110_inst_key
 Instant key receive function. Transmits an "instant" command to the keyboard and receives one byte
 in reply.
 Filters the received scancode through the same criteria as the recv_key but also differentiates
 between the numerical keypad keys and the special 'calc' keys (= / * +) which use the same scancodes
 as the arrow keys.
 This function is only called after a 'shift' key event.
 */
uint8_t m0110_inst_key(void)
{
    uint8_t key;

    key = m0110_inst_recv();

    if (key == M0110_PAD_CODE)    // If the reply is the keypad prefix, issue an 'immediate' command
    {                             // to receive the actual key scancode and...
      key = m0110_inst_recv();

   // If the scancode is one of the 'calc' keys add 0x60 to the key code
      if ((key&0x7F) == 0x05 || (key&0x7F) == 0x0D || \
          (key&0x7F) == 0x11 || (key&0x7F) == 0x1B)
        return ((key & 0x80) | ((key & 0x7F)>>1)) | M0110_ARR_ADD;
      else                                                  // otherwise, add the normal 0x40 for keypad
        return ((key & 0x80) | ((key & 0x7F)>>1)) | M0110_PAD_ADD;
    }
    else if (key == 0xFF || key == M0110_NULL)      // if the reply is null, return null
        return M0110_NULL;
    else                          // and if the reply is a normal key, return a normal key code
        return (key & 0x80) | ((key & 0x7F)>>1);
}

/*
 clock_lo
 Set the clock port to "low"
 */
static inline void clock_lo()
{
    M0110_CLOCK_DDR  |= M0110_CLOCK_SET;   //direction: output, set to 1
    M0110_CLOCK_PORT &= M0110_CLOCK_CLR;   //set bit to 0
}
/*
 clock_hi
 Set the clock port to "high"
 */
static inline void clock_hi()
{
    M0110_CLOCK_DDR  |= M0110_CLOCK_SET;   //direction: output, set to 1
    M0110_CLOCK_PORT |= M0110_CLOCK_SET;   //set bit to 1
}
/*
 clock_in
 Read the clock port state
 */
static inline bool clock_in()
{
    M0110_CLOCK_PORT |= M0110_CLOCK_SET;   //set bit to 1
    M0110_CLOCK_DDR  &= M0110_CLOCK_CLR;   //direction: input, set to 0
    _delay_us(1);
    return M0110_CLOCK_PIN & M0110_CLOCK_SET;
}
/*
 data_lo
 Set the data port to "low"
 */
static inline void data_lo()
{
    M0110_DATA_DDR  |= M0110_DATA_SET;    //direction: output, set to 1
    M0110_DATA_PORT &= M0110_DATA_CLR;    //set bit to 0
}
/*
 data_hi
 Set the data port to "high"
 */
static inline void data_hi()
{
    M0110_DATA_DDR  |= M0110_DATA_SET;    //direction: output, set to 1
    M0110_DATA_PORT |= M0110_DATA_SET;    //set bit to 1
}
/*
 data_in
 Read the data port state
 */
static inline bool data_in()
{
    M0110_DATA_PORT |= M0110_DATA_SET;    //set bit to 1
    M0110_DATA_DDR  &= M0110_DATA_CLR;    //direction: input, set to 0
    _delay_us(1);
    return M0110_DATA_PIN & M0110_DATA_SET;
}

/*
 wait_clock_lo
 Read the clock bit continuously, until it goes "low" or until timeout (us)
 */
static inline uint16_t wait_clock_lo(uint16_t us)
{
    while (clock_in()  && us) { asm(""); _delay_us(1); us--; }
    return us;
}
/*
 wait_clock_hi
 Read the clock bit continuously, until it goes "high" or until timeout (us)
 */
static inline uint16_t wait_clock_hi(uint16_t us)
{
    while (!clock_in() && us) { asm(""); _delay_us(1); us--; }
    return us;
}
/*
 Read the data bit continuously, until it goes "low" or until timeout (us)
 */
static inline uint16_t wait_data_lo(uint16_t us)
{
    while (data_in() && us)  { asm(""); _delay_us(1); us--; }
    return us;
}
/*
 Read the data bit continuously, until it goes "high" or until timeout (us)
 */
static inline uint16_t wait_data_hi(uint16_t us)
{
    while (!data_in() && us)  { asm(""); _delay_us(1); us--; }
    return us;
}

/*
 idle
 Set both clock and data bits to "high". Keyboard interprets that as "idle".
 */
static inline void idle(void)
{
    clock_hi();
    data_hi();
}

/*
 request
 Keeping the clock bit "high", pull the data bit "low. Keyboard interprets that as a request for
 a clock signal, so that a command can be sent to it from the controller (Teensy).
 */
static inline void request(void)
{
    clock_hi();
    data_lo();
}
