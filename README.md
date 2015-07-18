N-Way Blender and Weight Controller plugins for Maya
/**
 * @brief N-Way Blender and Weight Controller plugins for Maya
 * @section LICENSE The MIT License
 * @section requirements:  Eigen 3:  http://eigen.tuxfamily.org/
 * @section Autodesk Maya: http://www.autodesk.com/products/autodesk-maya/overview
 * @section (included) AffineLib: https://github.com/shizuo-kaji/AffineLib
 * @version 0.20
 * @date  18/Jul/2015
 * @author Shizuo KAJI
 */

(For details, look at the paper
Kaji, S. Tetrisation of triangular meshes and its application in shape blending. Proceedings of MEIS2015 (2015))
 
Two or more shapes are being blended according to the user specified weights.
Weights can be specified numerically or using the included Weight Controller plugin
based on the generalised barycentric weight.

How to compile:
* For Mac users, look at the included Xcode project file ( or Makefile )
* For Windows users, look at the included Visual Studio project file.
Please refer to Autodesk's web page for detail.

How to use:
1. Place the plugin files in "MAYA_PLUG_IN_PATH" (*.mll for windows, *.bundle for mac)
2. Place the UI python script files in "MAYA_SCRIPT_PATH" (ui_*.py)
3. Open Script editor in Maya and type in the following Python command:

    import ui_nwayBlender as ui
    ui.UI_NwayBlender()
    import ui_weightController
    ui_weightController.UI_WeightController()


First, create an N-Way blender deformer
by selecting (shift+click) target shapes and clicking "create" in the UI menu.
The last shape will be deformed.
Optionally, create a weight controller node by selecting control points
and clicking "create" in the UI.
Then, connect output of the controller to the input of N-Way blender by clicking "Out2Nway."
Now the target shape is deformed by moving the controller object around.
Look at the included jpg and video files for details.

To visualise ARAP energy, go to "Display" => "Polygon" => "Custom Polygon Display"
and tick "color" and select type "emission."

LIMITATION:
The ARAP version works only on "clean" meshes.
First apply "Cleanup" from "Mesh" menu
to remove zero area faces and zero length edges.

