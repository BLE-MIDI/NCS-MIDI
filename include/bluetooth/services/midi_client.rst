.. _midi_client_readme:

MIDI Service Client
################################

.. contents::
   :local:
   :depth: 2

The MIDI Service Client module can be used to interact with a connected peer that is running the GATT server with the :ref:`midi_service_readme`.
The client uses the :ref:`gatt_dm_readme` module to acquire the attribute handles that are necessary to interact with the I/O characteristic.

The NUS Service Client is used in the :ref:`central_midi` sample.


I/O Characteristic
*****************



API documentation
*****************

| Header file: :file:`include/bluetooth/services/midi_client.h`
| Source file: :file:`subsys/bluetooth/services/midi_client.c`

.. doxygengroup:: bt_midi_client
   :project: nrf_midi
   :members:
