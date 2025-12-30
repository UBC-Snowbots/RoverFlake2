# Makefile for common libraries

BUILD_DIR:=build

all: clean configure build test

configure:
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && cmake ..

build: configure 
	@$(MAKE) -C $(BUILD_DIR)

clean:
	@rm -rf $(BUILD_DIR)

test: configure build
	@${BUILD_DIR}/unit_tests


.PHONY: all configure build clean test