BUILD_DIR = build
FIRMWARE = $(BUILD_DIR)/ZfocFirmware.elf

include tup.config

ifneq (,$(findstring v1.,$(CONFIG_BOARD_VERSION)))
else
	$(error "Unsupported board version $(CONFIG_BOARD_VERSION)")
endif

$(info board version: $(CONFIG_BOARD_VERSION))

all:
	@tup --quiet -no-environ-check

clean:
	-rm -fR .dep $(BUILD_DIR)

.PHONY: all clean