# Copyright 2026 Mosaic SoC AG
# Solderpad Hardware License, Version 0.51, see LICENSE for details.
# SPDX-License-Identifier: SHL-0.51

# Generated job-file testbenches use the same build/run plumbing as handwritten TBs.
# Select a matching job name for every backend, or override the file for a single TB.
IDMA_VLT_JOB ?= simple
IDMA_VLT_JOB_FILE ?=

IDMA_VLT_BACKEND_TESTBENCHES := $(addprefix tb_idma_backend_,$(IDMA_BACKEND_IDS))
IDMA_VLT_TESTBENCHES += $(IDMA_VLT_BACKEND_TESTBENCHES)

define idma_vlt_define_backend
IDMA_VLT_SUFFIXES_tb_idma_backend_$(1) := default
# Disable the separate JSON DMA tracer; Verilator wave tracing remains available.
IDMA_VLT_ELAB_ARGS_tb_idma_backend_$(1)__default := -GDmaTracing=0
IDMA_VLT_RUN_DEPS_tb_idma_backend_$(1)__default := $(if $(IDMA_VLT_JOB_FILE),$(abspath $(IDMA_VLT_JOB_FILE)),$(abspath $(IDMA_ROOT)/jobs/backend_$(1)/$(IDMA_VLT_JOB).txt))
IDMA_VLT_RUN_ARGS_tb_idma_backend_$(1)__default := +job_file=$$(IDMA_VLT_RUN_DEPS_tb_idma_backend_$(1)__default)
endef

$(foreach backend,$(IDMA_BACKEND_IDS),$(eval $(call idma_vlt_define_backend,$(backend))))

.PHONY: idma_sim_vlt_backends
idma_sim_vlt_backends: $(addprefix idma_sim_vlt_,$(IDMA_VLT_BACKEND_TESTBENCHES))
