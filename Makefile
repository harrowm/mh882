SHELL     := bash
.SHELLFLAGS := -c

IV      := iverilog
VVP     := vvp
IVFLAGS := -g2012 -I rtl -I tb
SIM     := sim

RTL_SRCS := \
    rtl/m68882_cir_pkg.sv \
    rtl/m68882_sync.sv \
    rtl/m68882_biu.sv \
    rtl/m68882_cir.sv \
    rtl/m68882_top.sv

$(SIM):
	mkdir -p $(SIM)

$(SIM)/biu_smoke: $(RTL_SRCS) tb/m68882_biu_smoke_tb.sv | $(SIM)
	$(IV) $(IVFLAGS) -o $@ $^

.PHONY: test
test: $(SIM)/biu_smoke
	$(VVP) $(SIM)/biu_smoke

.PHONY: clean
clean:
	rm -rf $(SIM)
