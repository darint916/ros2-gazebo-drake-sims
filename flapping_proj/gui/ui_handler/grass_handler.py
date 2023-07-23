from PyQt6 import QtWidgets
from PyQt6.QtWidgets import QMessageBox

import os

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
        self.bind_elements_convert_5()
        # Assigning all the fields and names would be very ugly with in the __init__ method so
        # we assign the UI_CONFIG_NAME_TO_NORMAL_NAME and COMBO_BOX_SETTING_NAME_TO_INDEX in this method
        self.init_config_vars()

        # Main sdf file path for parsing init
        self.sdf_file_path = None

        #Calculated variables
        self._urdfFilePath = ""
        self._urdfFolderPath = ""

    def init_config_vars(self):
        self.UI_CONFIG_NAME_TO_NORMAL_NAME = {
            #   Original name,              Variable name,                  Type
            #  ["TAGS_QUERY",               self.pixiv_query_tags,          UI_TYPE.TEXT_INPUT],  
            ["", self.sdfPath, ]
        }
    
    #TODO: Parse in urdf file for links and joints
    def urdf_folder_file_bind(self):
        if self.useUrdfFolderCheckBox.isChecked():
            UiConfigurationHelper.browse_dir(self.urdfPath)
            self._urdfFolderPath = self.urdfPath.text()
            #get urdf file path
            self._urdfFilePath = os.path.join(self._urdfFolderPath, self._urdfFolderPath.split('/')[-1] + '.urdf')
        else:
            UiConfigurationHelper.browse_file(self.urdfPath)
            self._urdfFilePath = self.urdfPath.text()

            #check if file is urdf
            if not self._urdfFilePath.endswith('.urdf'):
                QMessageBox.critical(None, "Error", f"File is not a urdf file:\n{self._urdfFilePath}")
                return

        #check if file exists
        if not os.path.exists(self._urdfFilePath):
            QMessageBox.critical(None, "Error", f"Issue locating file. File does not exist:\n{self._urdfFilePath}")
            return

    '''
    TODO: Bind paths to variables
        Continue button needs to convert urdf to base sdf to prepare config for next window
        PATH binding checkbox needs to bind paths to variables
        continue button needs to verify, and then report errors if any
    '''
    def bind_elements_convert_1(self):
        self.browseUrdfButton.clicked.connect(lambda: self.urdf_folder_file_bind)
        self.browseSdfButton.clicked.connect(lambda: UiConfigurationHelper.browse_dir(self.sdfPath))
        

        self.continueButton1.clicked.connect(lambda: self.continue1_checks)
    
    '''
    Aerodynamics Page
    '''
    # def bind_elements_convert_5(self):
        # self.
        


      
