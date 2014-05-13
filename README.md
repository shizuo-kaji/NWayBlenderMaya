N-Way Blender and Weight Controller plugins for Maya
/**
 * @brief N-Way Blender and Weight Controller plugins for Maya
 * @section LICENSE The MIT License
 * @section requirements:  Eigen 3:  http://eigen.tuxfamily.org/
 * @section Autodesk Maya: http://www.autodesk.com/products/autodesk-maya/overview
 * @section (included) AffineLib: https://github.com/shizuo-kaji/AffineLib
 * @version 0.10
 * @date  1/Dec/2013
 * @author Shizuo KAJI
 */

Shapes are blended according to the user specified weights.
Weights can be specified numerically or using the included Weight Controller plugin
based on the generalised barycentric weight.

How to compile:
Look at the included Xcode project file.
For Windows users, please refer to Autodesk's web page.

How to use:
put the plugins in "MAYA_PLUG_IN_PATH"
put the python scripts in "MAYA_SCRIPT_PATH"
open script editor in Maya and type in the following Python command:
#
import ui_nwayBlender as ui
ui.UI_NwayBlender()
import ui_weightController
ui_weightController.UI_WeightController()
#

First, create an N-Way blender deformer by the UI.
And then, also create a controller node by the UI.
Finally, connect output of the controller to the input of N-Way blender by clicking "Out2Nway."
Now the target shape is deformed by moving the controller object around.
Look at the included jpg file.
