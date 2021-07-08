.. _central_midi:

Bluetooth: Central MIDI
#######################

.. contents::
   :local:
   :depth: 2

The Central MIDI sample demonstrates how to use the :ref:`_midi_client_readme`, as well as demonstrating how to handle the BLE-MIDI data.
It uses the MIDI Client to send data back and forth between a UART connection and a Bluetooth LE connection.


Overview
********

When connected, the sample parses, and encodes MIDI data received on the RX pin of the UART (arduino_serial) peripheral to the Bluetooth LE unit.
On Nordic Semiconductor's development kits.

MIDI data sent from the Bluetooth LE unit is decoded and the timestamps are interpreted before the data is sent out of the UART peripheral's TX pin.


.. _central_midi_debug:

Debugging
*********
Debug messages  are printed by the RTT logger.

If you want to view the debug messages, follow the procedure in :ref:`testing_rtt_connect`.

Requirements
************

The sample supports the following development kits:

.. table-from-rows:: /includes/sample_board_rows.txt
   :header: heading
   :rows: nrf52840dk_nrf52840, nrf52dk_nrf52832, nrf52833dk_nrf52833, nrf52833dk_nrf52820

The sample also requires another development kit running a compatible application (see :ref:`peripheral_midi`).

Building and running
********************
.. |sample path| replace:: :file:`samples/bluetooth/central_midi`

.. include:: /includes/build_and_run.txt




Dependencies
************

This sample uses the following |NCS| libraries:

* :ref:`midi_client_readme`
* :ref:`gatt_dm_readme`
* :ref:`nrf_bt_scan_readme`

In addition, it uses the following Zephyr libraries:

* ``include/zephyr/types.h``
* ``boards/arm/nrf*/board.h``
* :ref:`zephyr:kernel_api`:

  * ``include/kernel.h``

* :ref:`zephyr:api_peripherals`:

   * ``include/uart.h``

* :ref:`zephyr:bluetooth_api`:

  * ``include/bluetooth/bluetooth.h``
  * ``include/bluetooth/gatt.h``
  * ``include/bluetooth/hci.h``
  * ``include/bluetooth/uuid.h``
