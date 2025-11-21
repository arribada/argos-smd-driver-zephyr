# Makefile for building project documentation

# Set the documentation directory
DOC_DIR := docs

# Set the source directory for documentation files
DOC_SRC_DIR := $(DOC_DIR)

# Set the output directory for generated documentation
DOC_OUTPUT_DIR := $(DOC_DIR)/build

# Set the command to generate the documentation
DOC_GENERATOR := doxygen

# Set test outputs
TEST_DIR := twister-*

# Default target
all: docs

# Target to generate the documentation
docs:
	@mkdir -p $(DOC_OUTPUT_DIR)
	$(DOC_GENERATOR) $(DOC_SRC_DIR)/doxygen.config

test:
	$(clean)
	west twister -p adafruit_feather_nrf52840 -T tests/integration -c  --device-testing  --device-serial-pty scripts/uart-pty.py || true
	cat twister-out/adafruit_feather_nrf52840_nrf52840/zephyr/integration.argos_smd/handler.log

# Target to clean the generated documentation
clean:
	rm -rf $(DOC_OUTPUT_DIR) $(TEST_DIR)

.PHONY: all docs clean
