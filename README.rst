.. _ncs_midi:

nRF connect SDK MIDI
#######################

.. contents::
   :local:
   :depth: 2

This repository contains samples useful for developing MIDI applications in nRF connect SDK. 


Samples
********

Current samples:

bluetooth\\central_midi

bluetooth\\peripheral_midi

serial midi



Installing
**********

Make sure you have the toolpath for nRF connect SDK installed on your computer. This can be done via the nRF connect Toolchain manager.

Go to your desired folder and run the following command:


``west init -m https://github.com/BLE-MIDI/NCS-MIDI --mr main``

followed by:


``west update``


Learn more
**********
We've written an article about `Optimizing BLE-MIDI with regards to timing.  <https://devzone.nordicsemi.com/nordic/nordic-blog/b/blog/posts/optimizing-ble-midi-with-regards-to-timing-1293631358>`_ The article is written for this sample, and has some neat resources for those that wan't to learn more.
