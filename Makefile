ARDUINO_FQBN ?= arduino:avr:uno
ARDUINO_PORT ?= /dev/ttyUSB0
GPIO ?= 21
BUILD_DIR ?= /tmp/arduino-build-toyota-obd1

.PHONY: arduino-compile arduino-upload sim-self-test sim-dry-run hardware-test report wave clean-build

arduino-compile:
	arduino-cli compile --fqbn $(ARDUINO_FQBN) --build-path $(BUILD_DIR) arduino_code/ToyotaOBD1Receiver

arduino-upload:
	arduino-cli upload -p $(ARDUINO_PORT) --fqbn $(ARDUINO_FQBN) --input-dir $(BUILD_DIR)

sim-self-test:
	python3 python_code_aka_obd1_simulator/toyota_obd1_sim.py --self-test

sim-dry-run:
	python3 python_code_aka_obd1_simulator/toyota_obd1_sim.py --dry-run --count 2

hardware-test:
	python3 python_code_aka_obd1_simulator/hardware_loopback_test.py --port $(ARDUINO_PORT) --gpio $(GPIO) --count 3

report:
	pdflatex -interaction=nonstopmode -halt-on-error -output-directory report report/obd_report.tex
	pdflatex -interaction=nonstopmode -halt-on-error -output-directory report report/obd_report.tex

wave:
	gtkwave waveforms/hardware_loopback_gpio21.gtkw

clean-build:
	rm -rf $(BUILD_DIR)
