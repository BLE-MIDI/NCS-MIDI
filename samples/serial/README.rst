.. _serial_midi:

serial MIDI
###########

.. contents::
   :local:
   :depth: 2

The serial MIDI sample demonstrates how to receive, parse and send serial MIDI data using the UART peripheral. 


Overview
********

The sample receives a serial MIDI stream on the UART port, parses the data and sends it back out again. 
This is to demonstrate how to receive and send the data. It is not the most effective way to make a soft 
through MIDI-application. In a typical soft through application the parsing would be skipped and each byte 
would be transmitted directly.



.. _peripheral_midi_debug:

Debugging
*********

Debug messages are printed by the RTT logger.

If you want to view the debug messages, follow the procedure in :ref:`testing_rtt_connect`.

Requirements
************

The sample supports the following development kits:

nrf52840dk_nrf52840, nrf52dk_nrf52832, nrf52833dk_nrf52833, nrf52833dk_nrf52820


User interface
**************

LED 1:
   * Blinks with a period of 2 seconds, duty cycle 50%, when the main loop is running (device is advertising).

LED 2:
   * Switches when data is received.

Building and running
********************

``west build samples/serial -b [board name]``

``west flash``

Dependencies
************

This sample uses the following |NCS| libraries:

* :ref:`dk_buttons_and_leds_readme`

In addition, it uses the following Zephyr libraries:

* ``include/zephyr/types.h``
* ``boards/arm/nrf*/board.h``
* :ref:`zephyr:kernel_api`:

  * ``include/kernel.h``

* :ref:`zephyr:api_peripherals`:

   * ``incude/gpio.h``
   * ``include/uart.h``
