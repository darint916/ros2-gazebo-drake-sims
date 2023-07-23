from PyQt5.QtWidgets import QMessageBox

from gui.libs.ui_config_assist import UiConfigurationHelper


class IConfigWindowHandler:

    def __init__(self):
        self.UI_CONFIG_NAME_TO_NORMAL_NAME = None
        self.COMBO_BOX_SETTING_NAME_TO_INDEX = None

    def init_config_vars(self):
        raise NotImplementedError("init_config_var is not implemented, please implement it")

    def show_status_window(self):
        raise NotImplementedError("show_status_window is not implemented, please implement it")

    def show_config(self):
        """Show the config window. should be alias for window.show
        """
        raise NotImplementedError("show_config is not implemented, please implement it")

    def dump_config(self) -> dict:
        """Dump configuration

        Returns:
            dict: dictionary of configuration
        """
        return UiConfigurationHelper.dump_config(self.UI_CONFIG_NAME_TO_NORMAL_NAME,
                                                 self.COMBO_BOX_SETTING_NAME_TO_INDEX,
                                                 key_to_lower=True)

    def load_config(self, cfg_path: str):
        """Parse and load an pre-existing ini file to UI fields
        """
        cfg_dict = UiConfigurationHelper.parse_ini_config(cfg_path)

        UiConfigurationHelper.load_config(cfg_dict,
                                          self.UI_CONFIG_NAME_TO_NORMAL_NAME,
                                          self.COMBO_BOX_SETTING_NAME_TO_INDEX)

    def save_config(self, cfg_path: str, cfg_dict: dict):
        """Parse and write the data in the UI fields to an ini file
        """
        UiConfigurationHelper.save_config(cfg_dict, cfg_path)
        QMessageBox.information(None, "Success", f"Successfully exported current configuration to:\n{cfg_path}")