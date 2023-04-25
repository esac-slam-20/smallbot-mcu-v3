ROOT_DIR := .
APP_SRC_DIR := .
CMAKE_DIR = $(APP_SRC_DIR)

TARGET_DIR = keyboard/$(TARGET)
BUILD_DIR = _build

build : $(BUILD_DIR)/Makefile
	@echo Build 
	${MAKE} -C $(BUILD_DIR)

setup : $(BUILD_DIR)/Makefile

$(BUILD_DIR)/Makefile : 
	if [ ! -d $(BUILD_DIR) ] ; then mkdir $(BUILD_DIR) ; fi
	cd $(BUILD_DIR);  \
		cmake \
		    -G "Unix Makefiles" \
			-DCMAKE_TOOLCHAIN_FILE=../riscv.cmake \
		    ..

clean:
	rm -rf $(BUILD_DIR)

flash: build
	wchisp flash $(BUILD_DIR)/main.bin

.PHONY: setup build $(SUBTARGETS) $(SUBTARGETS_CLEAN) clean
.SUFFIXES:

