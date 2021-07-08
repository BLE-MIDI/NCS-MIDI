.. _peripheral_midi:

Bluetooth: Peripheral MIDI
##########################

.. contents::
   :local:
   :depth: 2

The Peripheral UART sample demonstrates how to use the :ref:`midi_service_readme`, as well as demonstrating how to handle the BLE-MIDI data.
It uses the MIDI service to send data back and forth between a UART connection and a Bluetooth LE connection.


Overview
********

When connected, the sample parses, and encodes MIDI data received on the RX pin of the UART (arduino_serial) peripheral to the Bluetooth LE unit.
On Nordic Semiconductor's development kits.

MIDI data sent from the Bluetooth LE unit is decoded and the timestamps are interpreted before the data is sent out of the UART peripheral's TX pin.


.. _peripheral_midi_debug:

Debugging
*********

Debug messages are printed by the RTT logger.

If you want to view the debug messages, follow the procedure in :ref:`testing_rtt_connect`.

Requirements
************

The sample supports the following development kits:

nrf52840dk_nrf52840, nrf52dk_nrf52832, nrf52833dk_nrf52833, nrf52833dk_nrf52820


The sample also requires a phone or tablet running a compatible application.

You can also test the application with the :ref:`central_midi` sample.
See the documentation for that sample for detailed instructions.

User interface
**************

LED 1:
   * Blinks with a period of 2 seconds, duty cycle 50%, when the main loop is running (device is advertising).

LED 2:
   * On when connected.

Building and running
********************

``west build samples/bluetooth/peripheral_midi -b [board name]``

``west flash``

Dependencies
************

This sample uses the following |NCS| libraries:

* :ref:`midi_service_readme`
* :ref:`dk_buttons_and_leds_readme`

In addition, it uses the following Zephyr libraries:

* ``include/zephyr/types.h``
* ``boards/arm/nrf*/board.h``
* :ref:`zephyr:kernel_api`:

  * ``include/kernel.h``

* :ref:`zephyr:api_peripherals`:

   * ``incude/gpio.h``
   * ``include/uart.h``

* :ref:`zephyr:bluetooth_api`:

  * ``include/bluetooth/bluetooth.h``
  * ``include/bluetooth/gatt.h``
  * ``include/bluetooth/hci.h``
  * ``include/bluetooth/uuid.h``