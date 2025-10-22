# ace.py

import json
import math
import logging
from . import ace_commun

# Константы
RECONNECT_COUNT = 10

def parse_rgb(s):
    """Преобразует строку 'R,G,B' в список [R, G, B]"""
    try:
        r, g, b = map(float, s.split(','))
        return [r, g, b]
    except Exception:
        raise ValueError("COLOR must be R,G,B")

class ACE:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')

        # Настройки подключения
        serial = config.get('serial', '/dev/ttyACM0')
        baud = config.getint('baud', 115200)
        self.ace_commun = ace_commun.AceCommun(serial, baud)

        # Пины датчиков
        extruder_sensor_pin = config.get('extruder_sensor_pin', None)
        # toolhead_sensor_pin = config.get('toolhead_sensor_pin', None)

        # Скорости и длины
        self.feed_speed = config.getint('feed_speed', 50)
        self.retract_speed = config.getint('retract_speed', 50)
        self.toolchange_retract_length = config.getint('toolchange_retract_length', 150)
        self.toolchange_load_length = config.getint('toolchange_load_length', 630)
        self.toolhead_sensor_to_nozzle_length = config.getint('toolhead_sensor_to_nozzle', 0)

        self.max_dryer_temperature = config.getint('max_dryer_temperature', 55)

        # Сохранённые переменные
        self.save_variables = self.printer.lookup_object('save_variables')
        self.variables = self.save_variables.allVariables

        # Endless spool
        saved_endless_spool_enabled = self.variables.get('ace_endless_spool_enabled', False)
        self.endless_spool_enabled = config.getboolean('endless_spool', saved_endless_spool_enabled)
        self.endless_spool_in_progress = False
        self.endless_spool_runout_detected = False
        self.change_tool_in_progress = False

        # Feed assist
        self.feed_assist_index = -1

        # Состояние
        self.toolhead = None
        self.ace_dev_fd = None
        self.connect_timer = None
        self.heatbeat_timer = None
        self.endless_spool_timer = None
        self.endstops = {}
        self.fw_info = {}
        self.reconneted_count = 0  # опечатка сохранена для совместимости

        # Инициализация инвентаря
        self._init_inventory()

        # Инициализация датчиков
        if extruder_sensor_pin:
            self._create_mmu_sensor(config, extruder_sensor_pin, "extruder_sensor")
        # if toolhead_sensor_pin:
        #     self._create_mmu_sensor(config, toolhead_sensor_pin, "toolhead_sensor")

        # Обработчики событий
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler("klippy:disconnect", self._handle_disconnect)

        # Регистрация G-код команд
        gcode = self.gcode
        gcode.register_command("ACE_SET_SLOT", self.cmd_ACE_SET_SLOT, desc=self.cmd_ACE_SET_SLOT_help)
        gcode.register_command("ACE_QUERY_SLOTS", self.cmd_ACE_QUERY_SLOTS)
        gcode.register_command("ACE_DEBUG", self.cmd_ACE_DEBUG)
        gcode.register_command("ACE_START_DRYING", self.cmd_ACE_START_DRYING)
        gcode.register_command("ACE_STOP_DRYING", self.cmd_ACE_STOP_DRYING)
        gcode.register_command("ACE_ENABLE_FEED_ASSIST", self.cmd_ACE_ENABLE_FEED_ASSIST)
        gcode.register_command("ACE_DISABLE_FEED_ASSIST", self.cmd_ACE_DISABLE_FEED_ASSIST)
        gcode.register_command("ACE_FEED", self.cmd_ACE_FEED)
        gcode.register_command("ACE_RETRACT", self.cmd_ACE_RETRACT)
        gcode.register_command("ACE_CHANGE_TOOL", self.cmd_ACE_CHANGE_TOOL)
        gcode.register_command("ACE_ENABLE_ENDLESS_SPOOL", self.cmd_ACE_ENABLE_ENDLESS_SPOOL)
        gcode.register_command("ACE_DISABLE_ENDLESS_SPOOL", self.cmd_ACE_DISABLE_ENDLESS_SPOOL)
        gcode.register_command("ACE_ENDLESS_SPOOL_STATUS", self.cmd_ACE_ENDLESS_SPOOL_STATUS)
        gcode.register_command("ACE_SAVE_INVENTORY", self.cmd_ACE_SAVE_INVENTORY)
        gcode.register_command("ACE_TEST_RUNOUT_SENSOR", self.cmd_ACE_TEST_RUNOUT_SENSOR)

        # Таймер heartbeat (остановлен до подключения)
        self.heatbeat_timer = self.reactor.register_timer(self._periodic_heartbeat_event, self.reactor.NEVER)

    def _init_inventory(self):
        # Слоты (статус, цвет и т.д.)
        self.info = {
            "status": "ready",
            "dryer": {
                "status": "stop",
                "target_temp": 0,
                "duration": 0,
                "remain_time": 0,
            },
            "temp": 0,
            "enable_rfid": 1,
            "fan_speed": 7000,
            "feed_assist_count": 0,
            "cont_assist_time": 0.0,
            "slots": [
                {"index": i, "status": "empty", "sku": "", "type": "", "color": [0, 0, 0]}
                for i in range(4)
            ],
        }

        # Инвентарь из сохранённых данных
        saved_inventory = self.variables.get("ace_inventory")
        if saved_inventory:
            self.inventory = [dict(slot) for slot in saved_inventory]
        else:
            self.inventory = [
                {"status": "empty", "color": [0, 0, 0], "material": "", "temp": 0}
                for _ in range(4)
            ]

    def _create_mmu_sensor(self, config, pin, name):
        section_name = f"filament_switch_sensor {name}"
        cfg = config.getsection(config.get_name()).section
        cfg.fileconfig.add_section(section_name)
        cfg.fileconfig.set(section_name, "switch_pin", pin)
        cfg.fileconfig.set(section_name, "pause_on_runout", "False")
        self.printer.load_object(config, section_name)

        # Добавление в query_endstops
        ppins = self.printer.lookup_object('pins')
        pin_params = ppins.parse_pin(pin, can_invert=True, can_pullup=True)
        share_name = f"{pin_params['chip_name']}:{pin_params['pin']}"
        ppins.allow_multi_use_pin(share_name)
        mcu_endstop = ppins.setup_pin('endstop', pin)

        query_endstops = self.printer.load_object(config, 'query_endstops')
        query_endstops.register_endstop(mcu_endstop, share_name)
        self.endstops[name] = mcu_endstop

    def _handle_ready(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        logging.info("ACE: Connecting to %s", self.ace_commun.name)
        self.connect_timer = self.reactor.register_timer(self._connect, self.reactor.NOW)

    def _handle_disconnect(self):
        logging.info("ACE: Closing connection %s", self.ace_commun.name)
        self._disconnect()

    def _connect(self, eventtime):
        try:
            self.gcode.respond_info("ACE: Try connecting ACE")
            err = self.ace_commun.connect()
            if err:
                raise err

            logging.info("ACE: Connected to %s", self.ace_commun.name)
            self.gcode.respond_info(f"ACE: Connected to {self.ace_commun.name}")

            # Регистрация FD в реакторе
            self.ace_dev_fd = self.reactor.register_fd(
                self.ace_commun.get_fd(),
                self.read_handle,
                self.write_handle
            )

            # Запрос информации о прошивке
            def fw_callback(response):
                self.fw_info = response.get("result", {})
                info_str = json.dumps(self.fw_info, separators=(',', ':'))
                self.gcode.respond_info(f"ACE: Firmware info {info_str}")

            self.send_request({"method": "get_info"}, fw_callback)

            # Запуск heartbeat
            if self.heatbeat_timer:
                self.reactor.update_timer(self.heatbeat_timer, self.reactor.NOW)

            # Запуск endless spool монитора
            if self.endless_spool_enabled:
                self.endless_spool_timer = self.reactor.register_timer(
                    self._endless_spool_monitor, eventtime + 1.0
                )

            # Восстановление feed assist при переподключении
            ace_current_index = self.variables.get("ace_current_index", -1)
            if ace_current_index != -1:
                self.gcode.respond_info(f"ACE: Re-enabling feed assist on reconnect for index {ace_current_index}")
                self._enable_feed_assist(ace_current_index)

            self.reconneted_count = 0
            return self.reactor.NEVER

        except Exception as e:
            logging.exception("ACE connect error")
            self._disconnect()
            if self.reconneted_count <= RECONNECT_COUNT:
                self.reconneted_count += 1
                delay = self.calc_reconnect_timeout(self.reconneted_count)
                self.gcode.respond_info(f"ACE: Will auto reconnect after {delay:.2f} s")
                return eventtime + delay
            else:
                self.gcode.respond_info("ACE: Reconnection exceeded the number of times, timeout exceeded 10 seconds.")
                return eventtime + 10.0

    def _disconnect(self):
        logging.info("ACE: Disconnecting...")
        self.gcode.respond_info("ACE: Disconnecting...")

        if self.heatbeat_timer:
            self.reactor.update_timer(self.heatbeat_timer, self.reactor.NEVER)
        if self.endless_spool_timer:
            self.reactor.unregister_timer(self.endless_spool_timer)
            self.endless_spool_timer = None
        if self.ace_dev_fd:
            self.reactor.set_fd_wake(self.ace_dev_fd, False, False)
            self.ace_dev_fd = None

        self.ace_commun.disconnect()

    def calc_reconnect_timeout(self, attempt):
        return 0.8 * attempt + math.cos(attempt) * 0.5

    def write_handle(self, eventtime):
        if self.ace_commun and self.ace_dev_fd:
            self.ace_commun.writer(eventtime)
            if self.ace_commun.is_send_queue_empty():
                self.reactor.set_fd_wake(self.ace_dev_fd, True, False)

    def read_handle(self, eventtime):
        try:
            err = self.ace_commun.reader(eventtime)
            if err:
                raise err
        except Exception as e:
            logging.exception("ACE read error")
            self._disconnect()
            if ("Respond timeout" in str(e) or "Unable to communicate" in str(e)) and self.reconneted_count <= RECONNECT_COUNT:
                self.reconneted_count += 1
                delay = self.calc_reconnect_timeout(self.reconneted_count)
                self.gcode.respond_info(f"ACE: Will auto reconnect after {delay:.2f} s")
                self.reactor.update_timer(self.connect_timer, eventtime + delay)

    def _periodic_heartbeat_event(self, eventtime):
        self.send_request({"method": "get_status"}, lambda resp: self._update_status(resp))
        return eventtime + 2.9839

    def _update_status(self, response):
        result = response.get("result")
        if isinstance(result, dict):
            self.info = result

    def dwell(self, delay):
        self.reactor.pause(self.reactor.monotonic() + delay)

    def send_request(self, request, callback):
        self.info["status"] = "busy"
        if self.ace_dev_fd:
            self.reactor.set_fd_wake(self.ace_dev_fd, True, True)
        self.ace_commun.push_send_queue(request, callback)

    def _check_endstop_state(self, name):
        endstop = self.endstops.get(name)
        if not endstop or not self.toolhead:
            return False
        print_time = self.toolhead.get_last_move_time()
        return endstop.query_endstop(print_time) > 0

    def wait_ace_ready(self):
        while self.info.get("status") != "ready":
            self.dwell(0.5)

    def _extruder_move(self, length, speed):
        pos = self.toolhead.get_position()
        pos[3] += length
        self.toolhead.move(pos, speed)
        return pos[3]

    def _endless_spool_monitor(self, eventtime):
        if not self.endless_spool_enabled or self.change_tool_in_progress or self.endless_spool_in_progress:
            return eventtime + 0.1

        current_tool = self.variables.get("ace_current_index", -1)
        if current_tool == -1:
            return eventtime + 0.1

        # Определяем, печатаем ли мы
        is_printing = False
        try:
            toolhead_status = self.toolhead.get_status(eventtime)
            if toolhead_status.get("homed_axes"):
                is_printing = True
        except:
            pass

        try:
            print_stats = self.printer.lookup_object("print_stats")
            if print_stats.get_status(eventtime).get("state") == "printing":
                is_printing = True
        except:
            pass

        try:
            idle_timeout = self.printer.lookup_object("idle_timeout")
            idle_state = idle_timeout.get_status(eventtime).get("state")
            if idle_state in ("Printing", "Ready"):
                is_printing = True
        except:
            pass

        # Всегда проверяем обрыв, если endless spool включён
        if current_tool >= 0:
            self._endless_spool_runout_handler()

        return eventtime + (0.05 if is_printing else 0.2)

    # --- Команды G-кода ---
    def cmd_ACE_START_DRYING(self, gcmd):
        temp = gcmd.get_int("TEMP", 0)
        duration = gcmd.get_int("DURATION", 240)
        if duration <= 0:
            raise gcmd.error("Wrong duration")
        if not (0 < temp <= self.max_dryer_temperature):
            raise gcmd.error("Wrong temperature")

        def callback(response):
            if response.get("code", 0) != 0:
                raise self.gcode.error(f"ACE Error: {response.get('msg')}")

        self.gcode.respond_info("ACE: Started ACE drying")
        self.send_request({
            "method": "drying",
            "params": {"temp": temp, "fan_speed": 7000, "duration": duration}
        }, callback)

    def cmd_ACE_STOP_DRYING(self, gcmd):
        def callback(response):
            if response.get("code", 0) != 0:
                raise self.gcode.error(f"ACE Error: {response.get('msg')}")
        self.gcode.respond_info("ACE: Stopped ACE drying")
        self.send_request({"method": "drying_stop"}, callback)

    def _enable_feed_assist(self, index):
        def callback(response):
            if response.get("code", 0) != 0:
                raise self.gcode.error(f"ACE Error: {response.get('msg')}")
            self.feed_assist_index = index
            self.gcode.respond_info("ACE: Enabled ACE feed assist")
        self.send_request({"method": "start_feed_assist", "params": {"index": index}}, callback)
        self.dwell(0.7)

    def cmd_ACE_ENABLE_FEED_ASSIST(self, gcmd):
        index = gcmd.get_int("INDEX")
        if not (0 <= index < 4):
            raise gcmd.error("Wrong index")
        self._enable_feed_assist(index)

    def _disable_feed_assist(self, index):
        def callback(response):
            if response.get("code", 0) != 0:
                raise self.gcode.error(f"ACE Error: {response.get('msg')}")
            self.feed_assist_index = -1
            self.gcode.respond_info("ACE: Disabled ACE feed assist")
        self.send_request({"method": "stop_feed_assist", "params": {"index": index}}, callback)
        self.dwell(0.3)

    def cmd_ACE_DISABLE_FEED_ASSIST(self, gcmd):
        index = gcmd.get_int("INDEX", self.feed_assist_index if self.feed_assist_index != -1 else -1)
        if not (0 <= index < 4):
            raise gcmd.error("Wrong index")
        self._disable_feed_assist(index)

    def _feed(self, index, length, speed):
        def callback(response):
            if response.get("code", 0) != 0:
                raise self.gcode.error(f"ACE Error: {response.get('msg')}")
        self.send_request({
            "method": "feed_filament",
            "params": {"index": index, "length": length, "speed": speed}
        }, callback)
        self.dwell(length / speed + 0.1)

    def cmd_ACE_FEED(self, gcmd):
        index = gcmd.get_int("INDEX")
        length = gcmd.get_int("LENGTH")
        speed = gcmd.get_int("SPEED", self.feed_speed)
        if not (0 <= index < 4):
            raise gcmd.error("Wrong index")
        if length <= 0:
            raise gcmd.error("Wrong length")
        if speed <= 0:
            raise gcmd.error("Wrong speed")
        self._feed(index, length, speed)

    def _retract(self, index, length, speed):
        def callback(response):
            if response.get("code", 0) != 0:
                raise self.gcode.error(f"ACE Error: {response.get('msg')}")
        self.send_request({
            "method": "unwind_filament",
            "params": {"index": index, "length": length, "speed": speed}
        }, callback)
        self.dwell(length / speed + 0.1)

    def cmd_ACE_RETRACT(self, gcmd):
        index = gcmd.get_int("INDEX")
        length = gcmd.get_int("LENGTH")
        speed = gcmd.get_int("SPEED", self.retract_speed)
        if not (0 <= index < 4):
            raise gcmd.error("Wrong index")
        if length <= 0:
            raise gcmd.error("Wrong length")
        if speed <= 0:
            raise gcmd.error("Wrong speed")
        self._retract(index, length, speed)

    def _feed_to_toolhead(self, tool):
        sensor = self.printer.lookup_object("filament_switch_sensor extruder_sensor")
        while not sensor.runout_helper.filament_present:
            self.wait_ace_ready()
            slot_status = self.info["slots"][tool]["status"]
            if slot_status == "ready":
                self._feed(tool, self.toolchange_load_length, self.retract_speed)
                self.variables["ace_filament_pos"] = "bowden"
                self.dwell(0.1)
            else:
                self.gcode.respond_info("Spool is empty")
                idle = self.printer.lookup_object("idle_timeout")
                if idle.get_status(self.reactor.monotonic())["state"] == "Printing":
                    self.gcode.run_script_from_command("PAUSE")
                raise self.gcode.error("Spool is empty")
        self.variables["ace_filament_pos"] = "spliter"
        self._enable_feed_assist(tool)
        self.variables["ace_filament_pos"] = "toolhead"
        self._extruder_move(self.toolhead_sensor_to_nozzle_length, 5)
        self.variables["ace_filament_pos"] = "nozzle"
        self.gcode.run_script_from_command("MOVE_THROW_POS")

    def cmd_ACE_CHANGE_TOOL(self, gcmd):
        tool = gcmd.get_int("TOOL")
        if not (0 <= tool < 4):
            raise gcmd.error("Wrong index")

        was = self.variables.get("ace_current_index", -1)
        if was == tool:
            gcmd.respond_info(f"ACE: Already tool {tool}")
            return

        if tool != -1 and self.info["slots"][tool]["status"] != "ready":
            gcmd.respond_info("ACE: Spool is empty")
            idle = self.printer.lookup_object("idle_timeout")
            if idle.get_status(self.reactor.monotonic())["state"] == "Printing":
                self.gcode.run_script_from_command("PAUSE")
            return

        endless_was = self.endless_spool_enabled
        if endless_was:
            self.endless_spool_enabled = False
            self.endless_spool_runout_detected = False
        self.change_tool_in_progress = True
        self.gcode.run_script_from_command(f"_ACE_PRE_TOOLCHANGE FROM={was} TO={tool}")

        try:
            if was != -1:
                self._disable_feed_assist(was)
                self.wait_ace_ready()
                pos = self.variables.get("ace_filament_pos", "spliter")
                if pos == "nozzle":
                    self.gcode.run_script_from_command("CUT_TIP")
                    self.variables["ace_filament_pos"] = "toolhead"
                if pos in ("toolhead", "nozzle"):
                    sensor = self.printer.lookup_object("filament_switch_sensor extruder_sensor")
                    while sensor.runout_helper.filament_present:
                        self._extruder_move(-50, 10)
                        self._retract(was, 100, self.retract_speed)
                        self.wait_ace_ready()
                    self.variables["ace_filament_pos"] = "bowden"
                self.wait_ace_ready()
                self._retract(was, self.toolchange_retract_length, self.retract_speed)
                self.variables["ace_filament_pos"] = "spliter"

            if tool != -1:
                self._feed_to_toolhead(tool)

            gcode_move = self.printer.lookup_object('gcode_move')
            gcode_move.reset_last_position()

            self.gcode.run_script_from_command(f"_ACE_POST_TOOLCHANGE FROM={was} TO={tool}")
            self.variables["ace_current_index"] = tool
            self.gcode.run_script_from_command(f"SAVE_VARIABLE VARIABLE=ace_current_index VALUE={tool}")
            self.gcode.run_script_from_command(f"SAVE_VARIABLE VARIABLE=ace_filament_pos VALUE='{self.variables['ace_filament_pos']}'")

        finally:
            self.change_tool_in_progress = False
            if endless_was:
                self.endless_spool_enabled = True

        gcmd.respond_info(f"ACE: Tool {tool} loaded")

    def _find_next_available_slot(self, current):
        for i in range(4):
            idx = (current + 1 + i) % 4
            if idx != current and self.inventory[idx]["status"] == "ready" and self.info["slots"][idx]["status"] == "ready":
                return idx
        return -1

    def _endless_spool_runout_handler(self):
        if not self.endless_spool_enabled or self.endless_spool_in_progress:
            return
        current = self.variables.get("ace_current_index", -1)
        if current == -1:
            return
        sensor = self.printer.lookup_object("filament_switch_sensor extruder_sensor")
        if not sensor:
            return
        present = sensor.runout_helper.filament_present
        triggered = self._check_endstop_state("extruder_sensor")
        if not present and not triggered:
            if not self.endless_spool_runout_detected:
                self.endless_spool_runout_detected = True
                self.gcode.respond_info("ACE: Endless spool runout detected, switching immediately")
                self._execute_endless_spool_change()

    def _execute_endless_spool_change(self):
        if self.endless_spool_in_progress:
            return
        current = self.variables.get("ace_current_index", -1)
        next_tool = self._find_next_available_slot(current)
        if next_tool == -1:
            self.gcode.respond_info("ACE: No available slots for endless spool, pausing print")
            self.gcode.run_script_from_command("PAUSE")
            self.endless_spool_runout_detected = False
            return

        self.endless_spool_in_progress = True
        self.endless_spool_runout_detected = False

        self.gcode.respond_info(f"ACE: Endless spool changing from slot {current} to slot {next_tool}")

        if current >= 0:
            self.inventory[current] = {"status": "empty", "color": [0,0,0], "material": "", "temp": 0}
            self.variables["ace_inventory"] = self.inventory
            self.gcode.run_script_from_command(f"SAVE_VARIABLE VARIABLE=ace_inventory VALUE='{json.dumps(self.inventory)}'")

        if current != -1:
            self._disable_feed_assist(current)
            self.wait_ace_ready()

        sensor = self.printer.lookup_object("filament_switch_sensor extruder_sensor")
        while sensor.runout_helper.filament_present:
            self._feed(next_tool, self.toolchange_load_length, self.retract_speed)
            self.wait_ace_ready()
            self.dwell(0.1)

        self._enable_feed_assist(next_tool)
        self.variables["ace_current_index"] = next_tool
        self.gcode.run_script_from_command(f"SAVE_VARIABLE VARIABLE=ace_current_index VALUE={next_tool}")

        self.endless_spool_in_progress = False
        self.gcode.respond_info(f"ACE: Endless spool completed, now using slot {next_tool}")

    def cmd_ACE_ENABLE_ENDLESS_SPOOL(self, gcmd):
        self.endless_spool_enabled = True
        self.variables["ace_endless_spool_enabled"] = True
        self.gcode.run_script_from_command("SAVE_VARIABLE VARIABLE=ace_endless_spool_enabled VALUE=true")
        gcmd.respond_info("ACE: Endless spool enabled")

    def cmd_ACE_DISABLE_ENDLESS_SPOOL(self, gcmd):
        self.endless_spool_enabled = False
        self.endless_spool_runout_detected = False
        self.endless_spool_in_progress = False
        self.variables["ace_endless_spool_enabled"] = False
        self.gcode.run_script_from_command("SAVE_VARIABLE VARIABLE=ace_endless_spool_enabled VALUE=false")
        gcmd.respond_info("ACE: Endless spool disabled")

    def cmd_ACE_ENDLESS_SPOOL_STATUS(self, gcmd):
        saved = self.variables.get("ace_endless_spool_enabled", False)
        gcmd.respond_info(f"ACE: Endless spool status:")
        gcmd.respond_info(f"  - Currently enabled: {self.endless_spool_enabled}")
        gcmd.respond_info(f"  - Saved enabled: {saved}")
        gcmd.respond_info(f"  - Runout detected: {self.endless_spool_runout_detected}")
        gcmd.respond_info(f"  - In progress: {self.endless_spool_in_progress}")

    def cmd_ACE_DEBUG(self, gcmd):
        method = gcmd.get("METHOD")
        params = gcmd.get("PARAMS", None)
        def callback(resp):
            self.gcode.respond_info(f"ACE: Response: {json.dumps(resp)}")
        if params:
            try:
                params = json.loads(params)
            except:
                raise gcmd.error("Invalid JSON in PARAMS")
            self.send_request({"method": method, "params": params}, callback)
        else:
            self.send_request({"method": method}, callback)

    def cmd_ACE_SET_SLOT(self, gcmd):
        idx = gcmd.get_int("INDEX")
        if not (0 <= idx < 4):
            raise gcmd.error("Invalid slot index")
        if gcmd.get_int("EMPTY", 0) == 1:
            self.inventory[idx] = {"status": "empty", "color": [0,0,0], "material": "", "temp": 0}
            self._save_inventory()
            gcmd.respond_info(f"ACE: Slot {idx} set to empty")
            return
        color_str = gcmd.get("COLOR")
        material = gcmd.get("MATERIAL")
        temp = gcmd.get_int("TEMP")
        if not color_str or not material or temp <= 0:
            raise gcmd.error("COLOR, MATERIAL, TEMP must be set unless EMPTY=1")
        color = parse_rgb(color_str)
        self.inventory[idx] = {"status": "ready", "color": color, "material": material, "temp": temp}
        self._save_inventory()
        gcmd.respond_info(f"ACE: Slot {idx} set: color={color}, material={material}, temp={temp}")

    def cmd_ACE_QUERY_SLOTS(self, gcmd):
        gcmd.respond_info(f"ACE: query slots: {json.dumps(self.inventory)}")

    def cmd_ACE_SAVE_INVENTORY(self, gcmd):
        self._save_inventory()
        gcmd.respond_info("ACE: Inventory saved to persistent storage")

    def _save_inventory(self):
        self.variables["ace_inventory"] = self.inventory
        self.gcode.run_script_from_command(f"SAVE_VARIABLE VARIABLE=ace_inventory VALUE='{json.dumps(self.inventory)}'")

    def cmd_ACE_TEST_RUNOUT_SENSOR(self, gcmd):
        sensor = self.printer.lookup_object("filament_switch_sensor extruder_sensor", None)
        if not sensor:
            gcmd.respond_info("ACE: Extruder sensor not found")
            return
        present = sensor.runout_helper.filament_present
        triggered = self._check_endstop_state("extruder_sensor")
        would_trigger = not present or not triggered
        gcmd.respond_info("ACE: Extruder sensor states:")
        gcmd.respond_info(f"  - Runout helper filament present: {present}")
        gcmd.respond_info(f"  - Endstop triggered: {triggered}")
        gcmd.respond_info(f"  - Endless spool enabled: {self.endless_spool_enabled}")
        gcmd.respond_info(f"  - Current tool: {self.variables.get('ace_current_index', -1)}")
        gcmd.respond_info(f"  - Runout detected: {self.endless_spool_runout_detected}")
        gcmd.respond_info(f"  - Would trigger runout: {would_trigger}")

    def get_status(self, eventtime):
        return {
            **self.info,
            "endless_spool": {
                "enabled": self.endless_spool_enabled,
                "runout_detected": self.endless_spool_runout_detected,
                "in_progress": self.endless_spool_in_progress,
            },
            "fw_info": self.fw_info,
        }

def load_config(config):
    return ACE(config)