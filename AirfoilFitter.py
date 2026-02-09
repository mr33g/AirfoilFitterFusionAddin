import adsk.core, adsk.fusion, adsk.cam, traceback
import os
import sys

# Get the directory where this script is located
addin_dir = os.path.dirname(__file__)

# Add the add-in directory and bundled 'lib' to the path
if addin_dir not in sys.path:
    sys.path.insert(0, addin_dir)

lib_dir = os.path.join(addin_dir, 'lib')
if lib_dir not in sys.path:
    sys.path.insert(0, lib_dir)

# Local imports after path setup
from utils.i18n import t

# Now we can import our modular components
from logic import state

def ensure_dependencies():
    """Ensures required libraries are available. Prioritizes bundled 'lib' folder."""
    try:
        import numpy
        import scipy
        import ezdxf
        return True
    except ImportError:
        pass

    ui = adsk.core.Application.get().userInterface
    res = ui.messageBox(
        t("deps_missing_msg"),
        t("deps_missing_title"),
        adsk.core.MessageBoxButtonTypes.YesNoButtonType
    )
    
    if res == adsk.core.DialogResults.DialogNo:
        return False
    
    try:
        import subprocess
        python_exe = sys.executable
        # Handle cases where Fusion uses a subfolder for Python
        potential_exe = os.path.join(os.path.dirname(python_exe), 'Python', 'python.exe')
        if os.path.exists(potential_exe):
            python_exe = potential_exe

        if not os.path.exists(lib_dir):
            os.makedirs(lib_dir)

        # Install directly into the add-in's lib folder
        pip_cmd = f'"{python_exe}" -m pip install --upgrade --force-reinstall --target "{lib_dir}" numpy scipy ezdxf'
        
        if os.name == 'nt':
            os.system(f'start "AirfoilFitter Dependency Installer" cmd /c "{pip_cmd} & pause"')
            ui.messageBox(t("deps_install_started"))
        else:
            subprocess.check_call([python_exe, '-m', 'pip', 'install', '--upgrade', '--force-reinstall', '--target', lib_dir, 'numpy', 'scipy', 'ezdxf'])
            ui.messageBox(t("deps_install_complete"))
            
        return False
    except Exception as e:
        ui.messageBox(t("deps_install_failed", error=str(e)))
        return False

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        # import debugpy
        # if not debugpy.is_client_connected():
        #     try:
        #         debugpy.listen(('localhost', 5678), in_process_debug_adapter=True)
        #         ui.messageBox('Debug server ready on port 5678.\nAttach VSCode now, then click OK.')
        #     except Exception as e:
        #         ui.messageBox(f'Debug setup warning: {str(e)}\nContinuing anyway...')        
        
        if not ensure_dependencies():
            return

        # Import these here, after dependencies are checked and potentially installed
        from ui.handlers import AirfoilFitterCommandCreatedHandler

        # 1. Create Command Definition
        cmd_def = ui.commandDefinitions.itemById('AirfoilFitterCommand')
        if not cmd_def:
            # Use absolute path for resources to be safe
            resource_path = os.path.join(addin_dir, 'resources', 'AirfoilFitterCommand')
            cmd_def = ui.commandDefinitions.addButtonDefinition(
                'AirfoilFitterCommand', 
                t("cmd_button_name"),
                t("cmd_button_desc"),
                resource_path
            )
            toolClip_path = os.path.join(addin_dir, 'resources', 'AirfoilFitterCommand', 'tooltip.png')
            cmd_def.toolClipFilename = toolClip_path
        
        on_command_created = AirfoilFitterCommandCreatedHandler()
        cmd_def.commandCreated.add(on_command_created)
        state.handlers.append(on_command_created)

        # 2. Add to INSERT panel in Solid and Surface workspaces
        # We try both common panel IDs: 'InsertPanel' and 'SolidInsertPanel'
        for ws_id, tab_id, panel_ids in [
            ('FusionSolidEnvironment', 'SolidTab', ['InsertPanel', 'SolidInsertPanel']),
            ('FusionSurfaceEnvironment', 'SurfaceTab', ['InsertPanel', 'SurfaceInsertPanel'])
        ]:
            workspace = ui.workspaces.itemById(ws_id)
            if workspace:
                tab = workspace.toolbarTabs.itemById(tab_id)
                if tab:
                    for p_id in panel_ids:
                        panel = tab.toolbarPanels.itemById(p_id)
                        if panel:
                            existing_control = panel.controls.itemById('AirfoilFitterCommand')
                            if not existing_control:
                                panel.controls.addCommand(cmd_def)
                            break # Found the panel, move to next workspace

    except:
        if ui:
            ui.messageBox(t("failed_to_start", error=traceback.format_exc()))

def stop(context):
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        
        # Cleanup from all possible panels
        for ws_id, tab_id, panel_ids in [
            ('FusionSolidEnvironment', 'SolidTab', ['InsertPanel', 'SolidInsertPanel']),
            ('FusionSurfaceEnvironment', 'SurfaceTab', ['InsertPanel', 'SurfaceInsertPanel'])
        ]:
            workspace = ui.workspaces.itemById(ws_id)
            if workspace:
                tab = workspace.toolbarTabs.itemById(tab_id)
                if tab:
                    for p_id in panel_ids:
                        panel = tab.toolbarPanels.itemById(p_id)
                        if panel:
                            control = panel.controls.itemById('AirfoilFitterCommand')
                            if control:
                                control.deleteMe()

        # Delete command definition
        cmd_def = ui.commandDefinitions.itemById('AirfoilFitterCommand')
        if cmd_def:
            cmd_def.deleteMe()
    except:
        pass
