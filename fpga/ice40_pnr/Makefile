all: top.rpt top.bin

top.json: ../LighthouseWrapperICE40.v
	yosys -p 'synth_ice40 -top LighthouseWrapperICE40 -json $@' $^
top.asc: top.json
	nextpnr-ice40 --lp8k --package cm81 --pcf pins.pcf --json $< --asc $@ --freq 32
top.bin: top.asc
	icepack $< $@
top.rpt: top.asc
	icetime -d lp8k -mtr $@ $^

prog: top.bin
	tinyprog -p $<

.PHONY: all %_waves prog clean
