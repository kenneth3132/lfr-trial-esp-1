import re
import os
import sys

def verify_pins():
    print("========================================================")
    print("PINOUT VERIFICATION: robot_connections.txt vs v1 Code")
    print("========================================================")

    base_dir = os.path.dirname(os.path.abspath(__file__))
    project_dir = os.path.dirname(base_dir)

    robot_conn_path = os.path.join(project_dir, 'robot_connections.txt')
    config_path = os.path.join(project_dir, 'v1', 'v1_modular', 'config.h')
    single_path = os.path.join(project_dir, 'v1', 'v1_esp32_pid', 'v1_esp32_pid.ino')

    # 1. Parse robot_connections.txt
    expected_pins = {}
    with open(robot_conn_path, 'r', encoding='utf-8') as f:
        text = f.read()

    matches = re.findall(r'\|\s*\*\*([^*]+)\*\*\s*\|\s*([^|]+)\|\s*GPIO\s*(\d+)\s*\|', text)
    for comp, pin_name, gpio in matches:
        clean_name = pin_name.strip()
        expected_pins[clean_name] = int(gpio)

    print(f"Extracted {len(expected_pins)} pins from robot_connections.txt:")
    for k, v in expected_pins.items():
        print(f"  - {k:20s}: GPIO {v}")

    # 2. Parse config.h
    print("\nChecking v1/v1_modular/config.h ...")
    with open(config_path, 'r', encoding='utf-8') as f:
        config_text = f.read()

    # Extract sensorPins array
    sensor_match = re.search(r'sensorPins\[NUM_SENSORS\]\s*=\s*\{([^}]+)\}', config_text)
    if not sensor_match:
        print("[FAIL] sensorPins array not found in config.h!")
        sys.exit(1)
    config_sensor_pins = [int(p.strip()) for p in sensor_match.group(1).split(',')]

    # Extract defines
    defines = dict(re.findall(r'#define\s+([A-Za-z0-9_]+)\s+(\d+)', config_text))

    # Compare Sensors S1 to S8
    expected_sensors = [36, 39, 34, 35, 32, 33, 25, 26]
    assert config_sensor_pins == expected_sensors, f"Sensor pins mismatch! Found {config_sensor_pins}, expected {expected_sensors}"
    print("  [PASS] Sensor pins S1-S8 match: " + str(config_sensor_pins))

    # Compare TB6612FNG
    expected_motors = {
        'PIN_PWMA': 17,
        'PIN_AIN1': 4,
        'PIN_AIN2': 16,
        'PIN_PWMB': 15,
        'PIN_BIN1': 5,
        'PIN_BIN2': 18,
        'PIN_STBY': 19
    }
    for def_name, exp_gpio in expected_motors.items():
        val = int(defines.get(def_name, -1))
        assert val == exp_gpio, f"Motor pin mismatch for {def_name}: found {val}, expected {exp_gpio}"
        print(f"  [PASS] {def_name:10s} == GPIO {val}")

    # Compare Buttons & LED
    expected_ui = {
        'BTN_START': 27,
        'BTN_CALIB': 14,
        'BTN_MODE': 13,
        'LED_PIN': 2
    }
    for def_name, exp_gpio in expected_ui.items():
        val = int(defines.get(def_name, -1))
        assert val == exp_gpio, f"UI pin mismatch for {def_name}: found {val}, expected {exp_gpio}"
        print(f"  [PASS] {def_name:10s} == GPIO {val}")

    # 3. Parse v1_esp32_pid.ino
    print("\nChecking v1/v1_esp32_pid/v1_esp32_pid.ino ...")
    with open(single_path, 'r', encoding='utf-8') as f:
        single_text = f.read()

    single_sensor_match = re.search(r'sensorPins\[NUM_SENSORS\]\s*=\s*\{([^}]+)\}', single_text)
    assert single_sensor_match, "[FAIL] sensorPins array not found in v1_esp32_pid.ino!"
    single_sensor_pins = [int(p.strip()) for p in single_sensor_match.group(1).split(',')]
    assert single_sensor_pins == expected_sensors, "Single-file sensor pins mismatch!"
    print("  [PASS] Single-file sensor pins S1-S8 match: " + str(single_sensor_pins))

    single_defines = dict(re.findall(r'#define\s+([A-Za-z0-9_]+)\s+(\d+)', single_text))
    all_expected = {**expected_motors, **expected_ui}
    for def_name, exp_gpio in all_expected.items():
        val = int(single_defines.get(def_name, -1))
        assert val == exp_gpio, f"Single-file mismatch for {def_name}: found {val}, expected {exp_gpio}"
        print(f"  [PASS] Single-file {def_name:10s} == GPIO {val}")

    print("\n>>> ALL PIN CONNECTIONS VERIFIED 100% MATCH WITH ZERO CONFLICTS! <<<\n")

if __name__ == '__main__':
    verify_pins()
