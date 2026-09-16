SHELL     := bash
.SHELLFLAGS := -c

IV      := iverilog
VVP     := vvp
IVFLAGS := -g2012 -I rtl -I tb
SIM     := sim

RTL_SRCS := \
    rtl/m68882_cir_pkg.sv \
    rtl/m68882_apu.sv \
    rtl/m68882_sync.sv \
    rtl/m68882_biu.sv \
    rtl/m68882_regfile.sv \
    rtl/m68882_proto.sv \
    rtl/m68882_cir.sv \
    rtl/m68882_top.sv

$(SIM):
	mkdir -p $(SIM)

$(SIM)/biu_smoke: $(RTL_SRCS) tb/m68882_biu_smoke_tb.sv | $(SIM)
	$(IV) $(IVFLAGS) -o $@ $^

$(SIM)/proto: $(RTL_SRCS) tb/m68882_proto_tb.sv | $(SIM)
	$(IV) $(IVFLAGS) -o $@ $^

$(SIM)/apu: $(RTL_SRCS) tb/m68882_apu_tb.sv | $(SIM)
	$(IV) $(IVFLAGS) -o $@ $^

$(SIM)/frame: $(RTL_SRCS) tb/m68882_frame_tb.sv | $(SIM)
	$(IV) $(IVFLAGS) -o $@ $^

$(SIM)/pipeline: $(RTL_SRCS) tb/m68882_pipeline_tb.sv | $(SIM)
	$(IV) $(IVFLAGS) -o $@ $^

# ── Phase 10: Conditional Predicate Field + BSUN ────────────────────────────
$(SIM)/cond: $(RTL_SRCS) tb/m68882_cond_tb.sv | $(SIM)
	$(IV) $(IVFLAGS) -o $@ $^

# ── Phase 7: Musashi golden-reference cosim ─────────────────────────────────
MUSASHI_DIR := tools/musashi
MUSASHI_SRC := $(MUSASHI_DIR)/m68kcpu.c $(MUSASHI_DIR)/m68kdasm.c \
               $(MUSASHI_DIR)/m68kops.c $(MUSASHI_DIR)/softfloat/softfloat.c
MUSASHI_FLAGS := -O2 -DM68K_EMULATE_FC=1 -I$(MUSASHI_DIR) -lm

$(MUSASHI_DIR)/m68kmake: $(MUSASHI_DIR)/m68kmake.c
	gcc -o $@ $<

$(MUSASHI_DIR)/m68kops.c $(MUSASHI_DIR)/m68kops.h: $(MUSASHI_DIR)/m68kmake
	cd $(MUSASHI_DIR) && ./m68kmake

tools/musashi_fpu_ref: tools/musashi_fpu_ref.c $(MUSASHI_SRC)
	gcc $(MUSASHI_FLAGS) -o $@ $^

tests/fpu_vectors.txt: scripts/gen_fpu_vectors.py
	python3 $<

tests/fpu_musashi_ref.txt: tools/musashi_fpu_ref tests/fpu_vectors.txt
	./tools/musashi_fpu_ref tests/fpu_vectors.txt > $@

$(SIM)/musashi_cosim: $(RTL_SRCS) tb/m68882_musashi_cosim_tb.sv | $(SIM)
	$(IV) $(IVFLAGS) -o $@ $^

# ── Phase 8: companion integration example ──────────────────────────────────
$(SIM)/glue: rtl/glue_cs_decode.sv tb/glue_cs_decode_tb.sv | $(SIM)
	$(IV) $(IVFLAGS) -o $@ $^

$(SIM)/example: $(RTL_SRCS) rtl/glue_cs_decode.sv example/mh882_companion_example.sv \
                tb/mh882_companion_example_tb.sv | $(SIM)
	$(IV) $(IVFLAGS) -I example -o $@ $^

.PHONY: test
test: $(SIM)/biu_smoke $(SIM)/proto $(SIM)/apu $(SIM)/frame $(SIM)/pipeline \
      $(SIM)/cond $(SIM)/musashi_cosim tests/fpu_musashi_ref.txt $(SIM)/glue $(SIM)/example
	$(VVP) $(SIM)/biu_smoke
	$(VVP) $(SIM)/proto
	$(VVP) $(SIM)/apu
	$(VVP) $(SIM)/frame
	$(VVP) $(SIM)/pipeline
	$(VVP) $(SIM)/cond
	$(VVP) $(SIM)/musashi_cosim
	$(VVP) $(SIM)/glue
	$(VVP) $(SIM)/example

.PHONY: clean
clean:
	rm -rf $(SIM)

.PHONY: clean-musashi
clean-musashi:
	rm -f $(MUSASHI_DIR)/m68kmake $(MUSASHI_DIR)/m68kops.c $(MUSASHI_DIR)/m68kops.h
	rm -f tools/musashi_fpu_ref
