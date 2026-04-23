import asyncio

import carb.settings
import omni.ext
import omni.ui as ui

from .constants import EXTENSION_ID, STATUS_IDLE, WINDOW_TITLE
from .task_runner import DualArmTaskController


class DualArmTaskExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._ext_id = ext_id
        self._controller = DualArmTaskController()

        self._window = ui.Window(WINDOW_TITLE, width=520, height=560)
        self._status_label = None
        self._step_label = None
        self._error_label = None
        self._start_button = None
        self._stop_button = None
        self._reload_button = None
        self._clear_log_button = None
        self._json_field = None
        self._log_field = None

        self._build_ui()
        self._controller.set_ui_hooks(
            on_status=self._update_status,
            on_step=self._update_step,
            on_error=self._update_error,
            on_running_changed=self._update_running_state,
            on_log=self._update_log,
        )
        self._refresh_from_settings()

    def on_shutdown(self):
        self._window = None
        self._controller = None

    def _build_ui(self):
        with self._window.frame:
            with ui.VStack(spacing=8, height=0):
                ui.Label("Dual Arm Task Control", height=24)
                ui.Separator(height=4)

                self._status_label = ui.Label(f"Status: {STATUS_IDLE}", height=20)
                self._step_label = ui.Label("Current Step: -", height=20)
                self._error_label = ui.Label("Last Error: None", word_wrap=True, height=60)

                ui.Spacer(height=4)
                ui.Label("JSON Path", height=18)
                self._json_field = ui.StringField(height=26)

                with ui.HStack(height=34, spacing=6):
                    self._start_button = ui.Button("Start Task", clicked_fn=self._on_start_clicked)
                    self._stop_button = ui.Button("Stop Task", clicked_fn=self._on_stop_clicked)
                    self._reload_button = ui.Button("Reload Settings", clicked_fn=self._on_reload_settings_clicked)

                with ui.HStack(height=34, spacing=6):
                    self._clear_log_button = ui.Button("Clear Log", clicked_fn=self._on_clear_log_clicked)
                    ui.Button("Reset Scene", clicked_fn=self._on_reset_scene_clicked)

                ui.Spacer(height=4)
                ui.Label("Debug Log", height=20)
                self._log_field = ui.StringField(height=220, multiline=True)
                self._log_field.enabled = False

                ui.Spacer(height=4)
                ui.Label(
                    "Tip: edit extension settings in config/extension.toml or Carb settings, then click Reload Settings.",
                    word_wrap=True,
                    height=52,
                )

    def _on_start_clicked(self):
        if self._controller is None or self._controller.is_running:
            return
        asyncio.ensure_future(self._start_task_flow())

    def _on_stop_clicked(self):
        if self._controller is None:
            return
        self._stop_and_reset()

    def _on_reset_scene_clicked(self):
        if self._controller is None:
            return
        self._reset_scene_state()

    def _on_reload_settings_clicked(self):
        text = self._json_field.model.get_value_as_string().strip() if self._json_field else ""
        if text:
            settings = carb.settings.get_settings()
            settings.set(f"/exts/{EXTENSION_ID}/json_path", text)
        self._controller.reload_config()
        self._refresh_from_settings()

    def _on_clear_log_clicked(self):
        if self._controller is not None:
            self._controller.clear_log()

    async def _start_task_flow(self):
        """
        Start 前先 reset 場景，再執行完整任務。
        """
        try:
            self._reset_scene_state()
            await asyncio.sleep(0.1)  # 讓 UI / event loop 有機會刷新，防止抖動
            await self._controller.run_full_task()
        except Exception as e:
            self._update_error(str(e))
            self._update_status("Error")

    def _stop_and_reset(self):
        """
        Stop 任務並重置場景。
        """
        try:
            # self._controller.stop()
            self._reset_scene_state()
        except Exception as e:
            self._update_error(str(e))
            self._update_status("Error")

    def _reset_scene_state(self):
        """
        場景重置邏輯放在 Extension / Controller 層，
        不直接塞進 task flow 本身。
        """
        if self._controller is None:
            return

        try:
            self._controller.stop()

            # 1. 解除 Lid attach / follow
            self._controller.detach_lid_if_needed()

            # 2. 恢復物理狀態
            self._controller.restore_lid_physics_if_needed()
            self._controller.restore_goods_physics_if_needed()

            # 3. Lid / goods 回到初始位置
            self._controller.reset_objects()

            # 4. 清掉剛體速度
            self._controller.zero_all_object_velocities()

            # 5. 手臂回 Home、夾爪打開
            self._controller.move_robots_home()
            self._controller.open_grippers()

            # 6. UI 狀態更新
            self._update_status(STATUS_IDLE)
            self._update_step("-")
            self._update_error("None")

            if hasattr(self._controller, "append_log"):
                self._controller.append_log("[Extension] Scene reset completed.")
        except Exception as e:
            self._update_error(str(e))
            self._update_status("Error")
            if hasattr(self._controller, "append_log"):
                self._controller.append_log(f"[Extension] Scene reset failed: {e}")

    def _refresh_from_settings(self):
        self._controller.reload_config()
        if self._json_field is not None:
            self._json_field.model.set_value(self._controller.config.json_path)
        self._update_status(self._controller.status)
        self._update_step(self._controller.current_step)
        self._update_error(self._controller.last_error)
        self._update_log(self._controller.log_text)
        self._update_running_state(self._controller.is_running)

    def _update_status(self, text: str):
        if self._status_label is not None:
            self._status_label.text = f"Status: {text}"

    def _update_step(self, text: str):
        if self._step_label is not None:
            self._step_label.text = f"Current Step: {text}"

    def _update_error(self, text: str):
        if self._error_label is not None:
            self._error_label.text = f"Last Error: {text}"

    def _update_log(self, text: str):
        if self._log_field is not None:
            self._log_field.model.set_value(text)

    def _update_running_state(self, is_running: bool):
        if self._start_button is not None:
            self._start_button.enabled = not is_running
            self._start_button.text = "Running..." if is_running else "Start Task"

        if self._stop_button is not None:
            self._stop_button.enabled = True

        if self._reload_button is not None:
            self._reload_button.enabled = not is_running