.. _midi_service_readme:

MIDI Service
#########################

.. contents::
   :local:
   :depth: 2

The Bluetooth LE GATT MIDI Service is a custom service that receives and writes MIDI data.

The NUS Service is used in the :ref:`peripheral_midi` sample.

Service UUID
************

The 128-bit vendor-specific service UUID is 03B80E5A-EDE8-4B33-A751-6CE34EC4C700  (16-bit offset: 0x0E5A).

Characteristics
***************

This service has one characteristic.

Data I/O Characteristic (7772E5DB-3868-4112-A1A9-F2669D106BF3)
========================================================

Write Without Response
   Write data to the I/O Characteristic to send it to the MIDI application.

Read
   Read  I/O Characteristic value.

Notify
   Enable notifications for the I/O Characteristic to receive data from the MIDI application.
   The application transmits all MIDI data as notifications.



API documentation
*****************

| Header file: :file:`include/bluetooth/services/midi.h`
| Source file: :file:`subsys/bluetooth/services/midi.c`

.. doxygengroup:: bt_midi
   :project: nrf_midi
   :members:
