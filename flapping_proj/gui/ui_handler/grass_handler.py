from PyQt6 import QtWidgets
from PyQt6.QtWidgets import QMessageBox


from gui.libs.i_config_window_handler import IConfigWindowHandler
from gui.ui_scripts.grass_ui import Ui_GrassMainWindow
from gui.libs.ui_config_assist import UI_TYPE, UiConfigurationHelper
# We inherit from the Ui class pyqt has generated from us from the Ui file so we can initialize it in this class
# We also "Implements" IConfigWindowHandler as it provides some functions that is common between configuration windows
class GrassGuiWindowHandler(Ui_GrassMainWindow, IConfigWindowHandler):
    def __init__(self):
        # Calls grass_ui constructor, it's useless because it does not have a constructor
        super().__init__()
        # We create an new QMainWindow instance to instantiate/setup the grass's ui onto
        self._window = QtWidgets.QMainWindow()
        # Instantiate window elements to the new MainWindow instance we just created
        self.setupUi(self._window)

        self.bind_elements_convert_1()

        # Assigning all the fields and names would be very ugly with in the __init__ method so
        # we assign the UI_CONFIG_NAME_TO_NORMAL_NAME and COMBO_BOX_SETTING_NAME_TO_INDEX in this method
        self.init_config_vars()

        # Main sdf file path for parsing init
        self.sdf_file_path = None

        #Calculated variables
        self.urdfFilePath = ""
        self.urdfFolderPath = ""

    def init_config_vars(self):
        self.UI_CONFIG_NAME_TO_NORMAL_NAME = {
            #   Original name,              Variable name,                  Type
            #  ["TAGS_QUERY",               self.pixiv_query_tags,          UI_TYPE.TEXT_INPUT],  
            ["", self.sdfPath, ]
        }
    
    def urdf_folder_file_bind(self):
        if self.useUrdfFolderCheckBox.isChecked():
            self.browseUrdfButton.clicked.connect(lambda: UiConfigurationHelper.browse_dir(self.urdfFolderPath))
        else:
            self.browseUrdfButton.clicked.connect(lambda: UiConfigurationHelper.browse_file(self.urdfFilePath))
    def bind_elements_convert_1(self):
        browseUrdfButton = self.browseUrdfButton.clicked.connect(lambda: self.urdf_folder_file_bind)
        



      
