# OpenWatchman FPGA

Exact timing of lighthouse signals implemented in RTL.

Currently only supports lighthouse 1.0 since that's all I have easy access to.

## Interface

This implementation acts as a SPI master. Every time an event happens, it writes data to the slave.

Communication is currently only one-way (from fpga to mcu).
