from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import sys
import time
import operator
import functools
import multiprocessing
import ifcopenshell

import OCC.Core.AIS

from collections import defaultdict, Iterable, OrderedDict



QString = str

os.environ['QT_API'] = 'pyqt5'
try:
    from pyqode.qt import QtCore
except BaseException:
    pass

from PyQt5 import QtCore, QtGui, QtWidgets



from ifcopenshell.geom.code_editor_pane import code_edit

try:
    from OCC.Display.pyqt5Display import qtViewer3d
except BaseException:
    import OCC.Display

    try:
        import OCC.Display.backend
    except BaseException:
        pass

    try:
        OCC.Display.backend.get_backend("qt-pyqt5")
    except BaseException:
        OCC.Display.backend.load_backend("qt-pyqt5")

    from OCC.Display.qtDisplay import qtViewer3d

from ifcopenshell.geom.main import settings, iterator
from ifcopenshell.geom.occ_utils import display_shape,set_shape_transparency

from ifcopenshell import open as open_ifc_file


if ifcopenshell.version < "0.6":
    # not yet ported
    from .. import get_supertype

from functions import *
import logging



class geometry_creation_signals(QtCore.QObject):
    completed = QtCore.pyqtSignal('PyQt_PyObject') #name of the signal completed
    progress = QtCore.pyqtSignal('PyQt_PyObject')


class geometry_creation_thread(QtCore.QThread):
    def __init__(self, signals, settings, f):
        QtCore.QThread.__init__(self)
        self.signals = signals
        self.settings = settings
        self.f = f

    def run(self):
        t0 = time.time()

        # detect concurrency from hardware, we need to have
        # at least two threads because otherwise the interface
        # is different
        # is different
        it = iterator(self.settings, self.f, max(2, multiprocessing.cpu_count()))
        if not it.initialize():
            self.signals.completed.emit([])
            return

        def _():

            old_progress = -1
            while True:
                shape = it.get()

                if shape:
                    yield shape

                if not it.next():
                    break

        self.signals.completed.emit((it, self.f, list(_())))


class configuration(object):
    def __init__(self):
        try:
            import ConfigParser
            Cfg = ConfigParser.RawConfigParser
        except BaseException:
            import configparser

            def Cfg():
                return configparser.ConfigParser(interpolation=None)

        conf_file = os.path.expanduser(os.path.join("~", ".ifcopenshell", "app", "snippets.conf"))
        if conf_file.startswith("~"):
            conf_file = None
            return

        self.config_encode = lambda s: s.replace("\\", "\\\\").replace("\n", "\n|")
        self.config_decode = lambda s: s.replace("\n|", "\n").replace("\\\\", "\\")

        if not os.path.exists(os.path.dirname(conf_file)):
            os.makedirs(os.path.dirname(conf_file))

        if not os.path.exists(conf_file):
            config = Cfg()
            config.add_section("snippets")
            config.set("snippets", "print all wall ids", self.config_encode("""
###########################################################################
# A simple script that iterates over all walls in the current model       #
# and prints their Globally unique IDs (GUIDS) to the console window      #
###########################################################################

for wall in model.by_type("IfcWall"):
    print ("wall with global id: "+str(wall.GlobalId))
""".lstrip()))

            config.set("snippets", "print properties of current selection", self.config_encode("""
###########################################################################
# A simple script that iterates over all IfcPropertySets of the currently #
# selected object and prints them to the console                          #
###########################################################################

# check if something is selected
if selection:
    #get the IfcProduct that is stored in the global variable 'selection'
    obj = selection
    for relDefinesByProperties in obj.IsDefinedBy:
         print("[{0}]".format(relDefinesByProperties.RelatingPropertyDefinition.Name))
         for prop in relDefinesByProperties.RelatingPropertyDefinition.HasProperties:
             print ("{:<20} :{}".format(prop.Name,prop.NominalValue.wrappedValue))
         print ("\\n")
""".lstrip()))
            with open(conf_file, 'w') as configfile:
                config.write(configfile)

        self.config = Cfg()
        self.config.read(conf_file)

    def options(self, s):
        return OrderedDict([(k, self.config_decode(self.config.get(s, k))) for k in self.config.options(s)])



class application(QtWidgets.QApplication):
    """A pythonOCC, PyQt based IfcOpenShell application
    with two tree views and a graphical 3d view"""

    class abstract_treeview(QtWidgets.QTreeWidget):

        """Base class for the two treeview controls"""

        instanceSelected = QtCore.pyqtSignal([object])
        instanceVisibilityChanged = QtCore.pyqtSignal([object, int])
        instanceDisplayModeChanged = QtCore.pyqtSignal([object, int])
        def __init__(self):
            QtWidgets.QTreeView.__init__(self)
            self.setColumnCount(len(self.ATTRIBUTES))
            self.setHeaderLabels(self.ATTRIBUTES)
            self.children = defaultdict(list)

        def get_children(self, inst):
            c = [inst]
            i = 0
            while i < len(c):
                c.extend(self.children[c[i]])
                i += 1
            return c

        def contextMenuEvent(self, event):
            menu = QtWidgets.QMenu(self)
            visibility = [menu.addAction("Show"), menu.addAction("Hide")]
            displaymode = [menu.addAction("Solid"), menu.addAction("Wireframe")]
            action = menu.exec_(self.mapToGlobal(event.pos()))
            index = self.selectionModel().currentIndex()
            inst = index.data(QtCore.Qt.UserRole)
            if hasattr(inst, 'toPyObject'):
                inst = inst
            if action in visibility:
                self.instanceVisibilityChanged.emit(inst, visibility.index(action))
            elif action in displaymode:
                self.instanceDisplayModeChanged.emit(inst, displaymode.index(action))

        def clicked_(self, index):
            inst = index.data(QtCore.Qt.UserRole)
            if hasattr(inst, 'toPyObject'):
                inst = inst
            if inst:
                self.instanceSelected.emit(inst)

        def select(self, product):
            print("select in the abstract_treeview called")
            itm = self.product_to_item.get(product)
            if itm is None:
                return
            self.selectionModel().setCurrentIndex(itm,
                                                  QtCore.QItemSelectionModel.SelectCurrent | QtCore.QItemSelectionModel.Rows)

    class decomposition_treeview(abstract_treeview):

        """Treeview with typical IFC decomposition relationships"""

        ATTRIBUTES = ['Entity', 'GlobalId', 'Name']

        def parent_overload(self, instance):
            if instance.is_a("IfcOpeningElement"):
                return instance.VoidsElements[0].RelatingBuildingElement
            if instance.is_a("IfcElement"):
                fills = instance.FillsVoids
                if len(fills):
                    return fills[0].RelatingOpeningElement
                containments = instance.ContainedInStructure
                if len(containments):
                    return containments[0].RelatingStructure
            if instance.is_a("IfcObjectDefinition"):
                decompositions = instance.Decomposes
                if len(decompositions):
                    return decompositions[0].RelatingObject

        def load_file(self, f, **kwargs):
            products = list(f.by_type("IfcProduct")) + list(f.by_type("IfcProject"))
            parents = list(map(self.parent_overload, products))
            items = {}
            skipped = 0
            ATTRS = self.ATTRIBUTES
            while len(items) + skipped < len(products):
                for product, parent in zip(products, parents):
                    if parent is None and not product.is_a("IfcProject"):
                        skipped += 1
                        continue
                    if (parent is None or parent in items) and product not in items:
                        sl = []
                        for attr in ATTRS:
                            if attr == 'Entity':
                                sl.append(product.is_a())
                            else:
                                sl.append(getattr(product, attr) or '')
                        itm = items[product] = QtWidgets.QTreeWidgetItem(items.get(parent, self), sl)
                        itm.setData(0, QtCore.Qt.UserRole, product)
                        self.children[parent].append(product)
            self.product_to_item = dict(zip(items.keys(), map(self.indexFromItem, items.values())))
            self.clicked.connect(self.clicked_)
            self.expandAll()

    class type_treeview(abstract_treeview):

        """Treeview with typical IFC decomposition relationships"""

        ATTRIBUTES = ['Name']

        def load_file(self, f, **kwargs):
            products = list(f.by_type("IfcProduct"))
            types = set(map(lambda i: i.is_a(), products))
            items = {}
            for t in types:
                def add(t):
                    s = get_supertype(t)
                    if s:
                        add(s)
                    s2, t2 = map(QString, (s, t))
                    if t2 not in items:
                        itm = items[t2] = QtWidgets.QTreeWidgetItem(items.get(s2, self), [t2])
                        itm.setData(0, QtCore.Qt.UserRole, t2)
                        self.children[s2].append(t2)

                if ifcopenshell.version < "0.6":
                    add(t)

            for p in products:
                t = QString(p.is_a())
                itm = items[p] = QtWidgets.QTreeWidgetItem(items.get(t, self), [p.Name or '<no name>'])
                itm.setData(0, QtCore.Qt.UserRole, t)
                self.children[t].append(p)

            self.product_to_item = dict(zip(items.keys(), map(self.indexFromItem, items.values())))
            self.clicked.connect(self.clicked_)
            self.expandAll()

    class property_table(QtWidgets.QWidget):

        def __init__(self):
            QtWidgets.QWidget.__init__(self)
            self.layout = QtWidgets.QVBoxLayout(self)
            self.setLayout(self.layout)
            self.scroll = QtWidgets.QScrollArea(self)
            self.layout.addWidget(self.scroll)
            self.scroll.setWidgetResizable(True)
            self.scrollContent = QtWidgets.QWidget(self.scroll)
            self.scrollLayout = QtWidgets.QVBoxLayout(self.scrollContent)
            self.scrollContent.setLayout(self.scrollLayout)
            self.scroll.setWidget(self.scrollContent)
            self.prop_dict = {}

        # triggered by selection event in either component of parent
        def select(self, product):
            print("select in the property_table called")
            # Clear the old contents if any
            while self.scrollLayout.count():
                child = self.scrollLayout.takeAt(0)
                if child is not None:
                    if child.widget() is not None:
                        child.widget().deleteLater()

            self.scroll = QtWidgets.QScrollArea()
            self.scroll.setWidgetResizable(True)

            prop_sets = self.prop_dict.get(str(product))

            if prop_sets is not None:
                for k, v in prop_sets:
                    group_box = QtWidgets.QGroupBox()

                    group_box.setTitle(k)
                    group_layout = QtWidgets.QVBoxLayout()
                    group_box.setLayout(group_layout)

                    for name, value in v.items():
                        prop_name = str(name)

                        value_str = value
                        if hasattr(value_str, "wrappedValue"):
                            value_str = value_str.wrappedValue

                        #if isinstance(value_str, unicode):
                        if isinstance(value_str, str):
                            value_str = value_str.encode('utf-8')
                        else:
                            value_str = str(value_str)

                        if hasattr(value, "is_a"):
                            type_str = " <i>(%s)</i>" % value.is_a()
                        else:
                            type_str = ""
                        label = QtWidgets.QLabel("<b>%s</b>: %s%s" % (prop_name, value_str, type_str))
                        group_layout.addWidget(label)

                    group_layout.addStretch()
                    self.scrollLayout.addWidget(group_box)

                self.scrollLayout.addStretch()
            else:
                label = QtWidgets.QLabel("No IfcPropertySets asscociated with selected entity instance")
                self.scrollLayout.addWidget(label)

        def load_file(self, f, **kwargs):
            for p in f.by_type("IfcProduct"):
                propsets = []

                def process_pset(prop_def):
                    if prop_def is not None:
                        prop_set_name = prop_def.Name
                        props = {}
                        if prop_def.is_a("IfcElementQuantity"):
                            for q in prop_def.Quantities:
                                if q.is_a("IfcPhysicalSimpleQuantity"):
                                    props[q.Name] = q[3]
                        elif prop_def.is_a("IfcPropertySet"):
                            for prop in prop_def.HasProperties:
                                if prop.is_a("IfcPropertySingleValue"):
                                    props[prop.Name] = prop.NominalValue
                        else:
                            # Entity introduced in IFC4
                            # prop_def.is_a("IfcPreDefinedPropertySet"):
                            for prop in range(4, len(prop_def)):
                                props[prop_def.attribute_name(prop)] = prop_def[prop]
                        return prop_set_name, props

                try:
                    for is_def_by in p.IsDefinedBy:
                        if is_def_by.is_a("IfcRelDefinesByProperties"):
                            propsets.append(process_pset(is_def_by.RelatingPropertyDefinition))
                        elif is_def_by.is_a("IfcRelDefinesByType"):
                            type_psets = is_def_by.RelatingType.HasPropertySets
                            if type_psets is None:
                                continue
                            for propset in type_psets:
                                propsets.append(process_pset(propset))
                except Exception as e:
                    import traceback
                    print("failed to load properties: {}".format(e))
                    traceback.print_exc()

                if len(propsets):
                    self.prop_dict[str(p)] = propsets

            print("property set dictionary has {} entries".format(len(self.prop_dict)))

    class viewer(qtViewer3d):

        instanceSelected = QtCore.pyqtSignal([object])

        @staticmethod
        def ais_to_key(ais_handle):
            def yield_shapes():
                ais = ais_handle.GetObject()
                if hasattr(ais, 'Shape'):
                    yield ais.Shape()
                    return
                shp = OCC.Core.AIS.Handle_AIS_Shape.DownCast(ais_handle)
                if not shp.IsNull():
                    yield shp.Shape()
                    return
                mult = ais_handle
                if mult.IsNull():
                    shp = OCC.Core.AIS.Handle_AIS_Shape.DownCast(ais_handle)
                    if not shp.IsNull():
                        yield shp
                else:
                    li = mult.GetObject().ConnectedTo()
                    for i in range(li.Length()):
                        shp = OCC.Core.AIS.Handle_AIS_Shape.DownCast(li.Value(i + 1))
                        if not shp.IsNull():
                            yield shp

            return tuple(shp.HashCode(1 << 24) for shp in yield_shapes())

        def __init__(self, widget):
            qtViewer3d.__init__(self, widget)
            self.ais_to_product = {}
            self.product_to_ais = {}
            self.counter = 0
            self.window = widget
            self.thread = None

            # ------------------------------------- Teng --------------------------------------------
            self.floor_elements_lst = []
            self.floor_compound_shapes_lst = []


        def initialize(self):
            self.InitDriver()

            #self._display.Select = self.HandleSelection

        def finished(self, file_shapes):
            it, f, shapes = file_shapes
            v = self._display

            t = {0: time.time()}

            def update(dt=None):
                t1 = time.time()
                if dt is None or t1 - t[0] > dt:
                    v.FitAll()
                    v.Repaint()
                    t[0] = t1

            for shape in shapes:
                ais = display_shape(shape, viewer_handle=v)
                product = f[shape.data.id]
                # try:
                #     ais.SetSelectionPriority(self.counter)
                # except:
                #     ais.GetObject().SetSelectionPriority(self.counter)
                self.ais_to_product[self.counter] = product
                self.product_to_ais[product] = ais
                self.counter += 1

                QtWidgets.QApplication.processEvents()

                if product.is_a() in {'IfcSpace', 'IfcOpeningElement'}:
                    v.Context.Erase(ais, True)

                #update(1.)

            update()

            self.thread = None

        # def load_file(self, f, setting=None):
        #
        #     if self.thread is not None:
        #         return
        #
        #     if setting is None:
        #         setting = settings()
        #         setting.set(setting.INCLUDE_CURVES, True)
        #         setting.set(setting.USE_PYTHON_OPENCASCADE, True)
        #
        #     self.signals = geometry_creation_signals()
        #     thread = self.thread = geometry_creation_thread(self.signals, setting, f)
        #     self.window.window_closed.connect(lambda *args: thread.terminate())
        #     self.signals.completed.connect(self.finished)
        #     self.thread.start()


        # change the load file in the viewer class make my own load_file

        def load_file(self, floor_name_lst, floor_elements_lst): # f is, ifc_file = ifcopenshell.open()
            time_a = time.time()
            #self.floor_elements_lst = floor_elements_lst

            settings = ifcopenshell.geom.settings()
            settings.set(settings.USE_PYTHON_OPENCASCADE, True)
            for i in range(len(floor_elements_lst)):
                # -------------- create shape ----------------:
                floor_ifc = floor_elements_lst[i]
                shapes = []
                if isinstance(floor_ifc, list):
                    # print("input is a list")
                    for element in floor_ifc:
                        try:
                            if element.Representation:
                                shape = ifcopenshell.geom.create_shape(settings, element).geometry
                                shapes.append(shape)
                        except:
                            print("Create shape failed, ", element.is_a(),',', element.Name)

                # -------------------create shape end -----------------------------

                shapes_compound, if_all_compound = list_of_shapes_to_compound(shapes)
                self.floor_compound_shapes_lst.append(shapes_compound)
                v = self._display
                if i != (len(floor_elements_lst)-1):
                    v.DisplayShape(shapes_compound, color="Black", transparency=0.9, update=False)
                    print("loading floor, index ",i, " floor name: ",floor_name_lst[i] )
                else:
                    v.DisplayShape(shapes_compound, color="Black", transparency=0.9, update=True)
                    print("loading floor, index ",i, " floor name: ",floor_name_lst[i] )

            print("load and create shape done, time: ", time.time()-time_a, "start,",time.time(),"end,", time_a)




        def select(self, product):
            print("select in viewer called")
            ais = self.product_to_ais.get(product)
            if ais is None:
                return
            v = self._display.Context
            v.ClearSelected(False)
            v.SetSelected(ais, True)

        def toggle(self, product_or_products, fn):
            if not isinstance(product_or_products, Iterable):
                product_or_products = [product_or_products]
            aiss = list(filter(None, map(self.product_to_ais.get, product_or_products)))
            last = len(aiss) - 1
            for i, ais in enumerate(aiss):
                fn(ais, i == last)

        def toggle_visibility(self, product_or_products, flag):
            v = self._display.Context
            if flag:
                def visibility(ais, last):
                    v.Erase(ais, last)
            else:
                def visibility(ais, last):
                    v.Display(ais, last)
            self.toggle(product_or_products, visibility)

        def toggle_wireframe(self, product_or_products, flag):
            v = self._display.Context
            if flag:
                def wireframe(ais, last):
                    if v.IsDisplayed(ais):
                        v.SetDisplayMode(ais, 0, last)
            else:
                def wireframe(ais, last):
                    if v.IsDisplayed(ais):
                        v.SetDisplayMode(ais, 1, last)
            self.toggle(product_or_products, wireframe)

        def HandleSelection(self, X, Y):
            print("HandleSelection called in viewer")
            v = self._display.Context
            v.Select(True)
            v.InitSelected()
            if v.MoreSelected():
                ais = v.SelectedInteractive()
                inst = self.ais_to_product[ais.GetOject().SelectionPriority()]
                self.instanceSelected.emit(inst)

    class window(QtWidgets.QMainWindow):

        TITLE = "IfcOpenShell IFC viewer"

        window_closed = QtCore.pyqtSignal([])

        def __init__(self):
            QtWidgets.QMainWindow.__init__(self)
            self.setWindowTitle(self.TITLE)
            self.menu = self.menuBar()
            self.menus = {}

        def closeEvent(self, *args):
            self.window_closed.emit()

        def add_menu_item(self, menu, label, callback, icon=None, shortcut=None):
            m = self.menus.get(menu)
            if m is None:
                m = self.menu.addMenu(menu)
                self.menus[menu] = m

            if icon:
                a = QtWidgets.QAction(QtGui.QIcon(icon), label, self)
            else:
                a = QtWidgets.QAction(label, self)

            if shortcut:
                a.setShortcut(shortcut)

            a.triggered.connect(callback)
            m.addAction(a)

    def makeSelectionHandler(self, component):
        print("makeSelectionHandler")
        def handler(inst):
            for c in self.components:
                if c != component:
                    c.select(inst)

        return handler

    def __init__(self, settings=None):
        QtWidgets.QApplication.__init__(self, sys.argv)
        self.window = application.window()
        self.tree = application.decomposition_treeview()
        self.tree2 = application.type_treeview()
        self.propview = self.property_table()
        self.canvas = application.viewer(self.window)
        self.tabs = QtWidgets.QTabWidget()
        self.window.resize(800, 600)

# --------------------------------------------------
        #self.pbar = QtWidgets.QProgressBar()
        #self.pbar_value = 0

# --------------------------------------------------


        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        splitter.addWidget(self.tabs)
        self.tabs.addTab(self.tree, 'Decomposition')
        self.tabs.addTab(self.tree2, 'Types')
        self.tabs.addTab(self.propview, "Properties")

        splitter2 = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        splitter2.addWidget(self.canvas)
        # self.editor = code_edit(self.canvas, configuration().options('snippets'))
        #splitter2.addWidget(self.pbar)
        splitter.addWidget(splitter2)
        splitter.setSizes([200, 600])

        #splitter2.setSizes([590,10])

        self.window.setCentralWidget(splitter)
        self.canvas.initialize()
        #self.components = [self.tree, self.tree2, self.canvas, self.propview, self.editor]
        self.components = [self.tree, self.tree2, self.propview]
        self.files = {}

        self.window.add_menu_item('File', '&Open', self.browse, shortcut='CTRL+O')
        self.window.add_menu_item('File', '&Close', self.clear, shortcut='CTRL+W')
        self.window.add_menu_item('File', '&Exit', self.window.close, shortcut='ALT+F4')

        #-----------------------add Teng's function -------------------------------



        # ---------------------------variables -----------------------------------
        self.s = 0.2
        self.dbscan =2
        self.k = 16
        self.floornum = int(0)
        self.current_ifc_file = None
        self.floor_elements_lst = []
        self.floor_name_lst = []
        self.base_polygon = None
        self.base_overhang_obb_poly = None
        self.base_obb_pt_lst = []
        self.base_overhang_points =None
        self.base_floor_num = 1
        self.overhang_left = True

        self.storeyElevation_lst=[]



        self.addGeoreference = False
        self.georeference_x = 0.0
        self.georeference_y = 0.0
        self.georeference_z = 0.0

        # functions

        # View
        self.window.add_menu_item('View', '&ClearViewer', self.clearViewer)
        self.window.add_menu_item('View', '&Showfloor', self.showfloor)
        self.window.add_menu_item('View', '&Show all floors', self.showallfloor)

        # preprocess
        self.window.add_menu_item('Preprocess','&Set base floor number',self.setBaseFloornum)
        self.window.add_menu_item('Preprocess', '&Add Georeference Point', self.addGeoreferencePoint)
        self.window.add_menu_item('Preprocess', '&Set Overhang Direction', self.setOverhangdir)
        self.window.add_menu_item('Preprocess', '&Set Overlap Parameters', self.setOverlapParameters)

        # Overlap
        self.window.add_menu_item('Overlap','&Single Floor Overlap',self.OverlapOneFloor)
        self.window.add_menu_item('Overlap','&Single Floor Overlap Bounding Box',self.OverlapOneFloorOBB)
        self.window.add_menu_item('Overlap','&All Floor Overlap',self.OverlapAll)
        self.window.add_menu_item('Overlap','&All Floor Overlap Bounding Box',self.OverlapAllOBB)

        # Overhang
        self.window.add_menu_item('Overhang','&Single Floor Overhang',self.OverhangOneFloor)
        self.window.add_menu_item('Overhang', '&All Floor Overhang', self.OverhangAll_new)



        self.window.add_menu_item('Height','&Height',self.GetHeight)
        self.window.add_menu_item('Height','&GetBaseHeight',self.GetBaseHeight)

        self.window.add_menu_item('WKT', '&Write Floor Footprint to WKT', self.footprintWKT)

        self.window.add_menu_item('Parking', '&Parking Units Calculation', self.parkingCalcualate, shortcut='CTRL+F')




        # ----------------------------end------------------------------------------


        self.tree.instanceSelected.connect(self.makeSelectionHandler(self.tree))
        self.tree2.instanceSelected.connect(self.makeSelectionHandler(self.tree2))
        self.canvas.instanceSelected.connect(self.makeSelectionHandler(self.canvas)) # handle selection????
        for t in [self.tree, self.tree2]:
            t.instanceVisibilityChanged.connect(functools.partial(self.change_visibility, t))
            t.instanceDisplayModeChanged.connect(functools.partial(self.change_displaymode, t))

        self.settings = settings

    def change_visibility(self, tree, inst, flag):
        insts = tree.get_children(inst)
        self.canvas.toggle_visibility(insts, flag)

    def change_displaymode(self, tree, inst, flag):
        insts = tree.get_children(inst)
        self.canvas.toggle_wireframe(insts, flag)

    def start(self):
        self.window.show()
        sys.exit(self.exec_())

    def browse(self):
        filename = QtWidgets.QFileDialog.getOpenFileName(self.window, 'Open file', ".",
                                                         "Industry Foundation Classes (*.ifc)")[0]
        if filename:
            self.load(filename)

    def clear(self):
        self.canvas._display.Context.RemoveAll(True)
        self.tree.clear()
        self.files.clear()

    def load(self, fn):
        if fn in self.files:
            return



        f = open_ifc_file(str(fn))  # f is, ifc_file = ifcopenshell.open()

        #Run the floor segementation when loading
        self.floor_elements_lst, self.floor_name_lst = GetElementsByStorey(f) #fast


        self.files[fn] = f
        for c in self.components:
            c.load_file(f, setting=self.settings)

        storeys = f.by_type("IfcBuildingStorey")
        for st in storeys:
            self.storeyElevation_lst.append(st.Elevation/1000.0)

        # new viewer load function
        self.canvas.load_file(self.floor_name_lst,self.floor_elements_lst)

        if not os.path.exists('./result'):
            os.makedirs('./result/')


    # ----------------------- teng's function -----------------------------
    def fun1(self):
        print("fun1")
        dialog = ParkingInputDialog()
        if dialog.exec():
            print(dialog.getInputs())
        #self.canvas._display.Context.RemoveAll(True)
        pass

    def showShapes(self,shapes):
        v = self.canvas._display
        for shape in shapes:
            ais = display_shape(shape, viewer_handle=v)
            v.FitAll()
            v.Repaint()

    def setBaseFloornum(self):
        floor_num, okPressed = QtWidgets.QInputDialog.getInt(self.window, 'Set Base floor number', 'Base Floor Number=')
        if okPressed:
            # self.canvas._display.Context.RemoveAll(True)
            self.base_polygon = None

            self.base_floor_num = floor_num
            print("Base floor num has been set to, " ,self.base_floor_num)
        else:
            return
        if self.floor_name_lst:
            msg = QtWidgets.QMessageBox()
            msg.setIcon(QtWidgets.QMessageBox.Information)
            msg.setText("Base floor number is has been set to, "+ str(floor_num)+ "  floor name: "+self.floor_name_lst[floor_num] )
            msg.setWindowTitle("Floor number Error")
            msg.setStandardButtons(QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel)
            msg.show()
            msg.exec_()

    def clearViewer(self):
        self.canvas._display.Context.EraseAll(True)

    def showallfloor(self):
        v = self.canvas._display
        for i in range(len(self.floor_name_lst)):
            v.DisplayShape(self.canvas.floor_compound_shapes_lst[i], color="WHITE", transparency=0.8, update=True)



    def GetBaseHeight(self):
        if not self.floor_name_lst:
            return
        floor_num, okPressed = QtWidgets.QInputDialog.getInt(self.window, 'Base Floor number', 'FloorNumber=')
        if okPressed:
            print("Floor name, ", self.floor_name_lst[floor_num] )
            top_height = float("{:.3f}".format(self.storeyElevation_lst[floor_num+1]))

            msg = QtWidgets.QMessageBox()
            msg.setIcon(QtWidgets.QMessageBox.Information)
            msg.setText("Base height is, "+str(top_height)+ " meter\n"+ "Floor name is ,"+self.floor_name_lst[floor_num])
            msg.setWindowTitle("Base height")
            msg.setStandardButtons(QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel)
            msg.show()
            msg.exec_()

        else:
            return





    def showfloor(self):
        if not self.floor_name_lst:
            return

        floor_num, okPressed =QtWidgets.QInputDialog.getInt(self.window,'InputFloorNumber','FloorNumber=')
        if okPressed:
            #self.canvas._display.Context.RemoveAll(True)

            self.floornum = floor_num
        else:
            return

        if floor_num>=0 and floor_num < len(self.floor_elements_lst):
            v = self.canvas._display
            #v.EraseAll()
            #v.Context.RemoveAll(True)

            print("Selected floor number:", floor_num, "  floor name: ", self.floor_name_lst[floor_num])


            # for floor_elements in self.floor_elements_lst:
            #     self.canvas.toggle_visibility(floor_elements, False)
            # self.canvas.toggle_visibility(self.floor_elements_lst[floor_num], True)
            v.DisplayShape(self.canvas.floor_compound_shapes_lst[floor_num], color="WHITE", transparency=0.8, update=True)

            # for shape in shapes:
            #     v.DisplayShape(shape, color="WHITE", transparency=0.8, update=True)  # grey


        else:
            print("message")
            msg = QtWidgets.QMessageBox()
            msg.setIcon(QtWidgets.QMessageBox.Critical)
            msg.setText("Floor number is out of range")
            msg.setWindowTitle("Floor number Error")
            msg.setStandardButtons(QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel)
            msg.show()
            msg.exec_()





    def GetFloorPolygon(self, i, filepath, yamlFilepath):
        #--------------------------------------------------------
        #f =open(filepath, "w+")


        #floor_elements = self.floor_elements_lst[i]
        floor_name = self.floor_name_lst[i]
        print("current floor, ", floor_name,"******************************************************************************")



        #display shapes
        v = self.canvas._display

        # for shape in shapes:

        #v.DisplayShape(self.canvas.floor_compound_shapes_lst[i], color="WHITE", transparency=0.8, update=False)

        # -------------------------------------z_min z_max ----------------------------------------------------------------
        # # get 3d oriented BBox of the shapes
        # aBox, center_pt, [aHalfX, aHalfY, aHalfZ], corners_bot, corners_top = GetOrientedBoundingBoxShapeCompound_old(self.canvas.floor_compound_shapes_lst[i])
        #
        # pyocc_corners_list = []
        # for pt in corners_top:
        #     pyocc_corners_list.append([pt.X(), pt.Y()])
        #     # print([pt.X(), pt.Y()])
        # poly_corners = Polygon(pyocc_corners_list)
        #
        # x_max, x_min, y_max, y_min, z_max, z_min = GetCornerMaxMin(corners_bot, corners_top)
        # print("z_max,z_min,", z_max,z_min)
        # if corners_bot[0].Z() == corners_top[0].Z():
        #     print("bot,top, z values,", z_min, z_max)
        # else:
        #     print("2. bot,top, z values,", corners_bot[0].Z(), corners_top[0].Z())

        # cutting height
        # cutting_height = z_min + 1.5 # Boompjes +1.5
        # ------------------------------------------------------------------------------------------------------------


        # parameters
        s = self.s
        dbscan = self.dbscan
        k = self.k


        calcconvexhull = False
        use_obb= False

        # ------------------- Elevation cutting_height ------------------------------------
        cutting_height = self.storeyElevation_lst[i] + 1.0

        #yml_file = open('./parameters.yaml', 'r')
        yml_file = open(yamlFilepath, 'r')
        yml_data = yaml.load(yml_file, Loader=Loader)
        str1 = "f"+str(i)
        if str1 in yml_data.keys():
            dict2 = yml_data[str1]
            if 'cutting_height' in dict2.keys():
                value = float(dict2['cutting_height'])
                cutting_height = self.storeyElevation_lst[i] + value

            if 'k' in dict2.keys():
                k = float(dict2['k'])
            if 'use_obb' in dict2.keys():
                if dict2['use_obb'] == True:
                    use_obb=True
            if 's' in dict2.keys():
                s = float(dict2['s'])
            if 'dbscan' in dict2.keys():
                dbscan = float(dict2['dbscan'])
            if 'calcconvexhull' in dict2.keys():
                if dict2['calcconvexhull'] == True:
                    calcconvexhull = True

        if use_obb:
            print("use_obb, ", use_obb, "floor name,", floor_name,
                  " ----------------------------------------------------------------------")
            pts = GetOrientedBoundingBoxShapeCompound(self.canvas.floor_compound_shapes_lst[i], False)
            Z_value = []
            for pt in pts:
                Z_value.append(pt.Z())
            z_max = max(Z_value)
            z_min = min(Z_value)
            z_mid = 0.5 * (z_max + z_min)
            pts_low = []
            pts_up = []
            for pt in pts:
                if pt.Z() < z_mid:
                    pts_low.append(pt)
                else:
                    pts_up.append(pt)

            corners_top = pts_up

            pyocc_corners_list = []
            for pt in corners_top:
                pyocc_corners_list.append([float("{:.3f}".format(pt.X() )), float("{:.3f}".format(pt.Y() ))])

            points = np.array(pyocc_corners_list)

            obb_hull = ConvexHull(points)
            result = []
            for idx in obb_hull.vertices:
                result.append(pyocc_corners_list[idx])
            poly_footprint = Polygon(result)
            return [poly_footprint]




        print("cutting height,", cutting_height)
        section_shape = GetSectionShape(cutting_height, self.canvas.floor_compound_shapes_lst[i])


        v.DisplayShape(section_shape, color="RED", update=True)

        # get the section shape edges
        edges = GetShapeEdges(section_shape)
        if s !=0:
            first_xy = GetEdgeSamplePointsPerDistance(edges, s)
        else:
            first_xy = GetEdges2DPT(edges)
        np_points = np.array(first_xy)


        corners = GetNumpyOBB(np_points, calcconvexhull=calcconvexhull, show_plot=False)

        OBB_poly = Polygon(corners.tolist())


        # create result dir
        if not os.path.exists('./result/Overlap/' + floor_name):
            os.makedirs('./result/Overlap/' + floor_name)

        img_filepath = "./result/Overlap/" + floor_name + "/obbAndPoints.png"
        SavePloyAndPoints(OBB_poly, np_points, color='b', filepath=img_filepath)
        # PlotPolyAndPoints(OBB_poly, np_points, color='b')
        # DBSCANClustering

        cluster_filepath = "./result/Overlap/" + floor_name + "/clusters.png"
        cluster_lst = GetDBSCANClusteringlst(np_points, dbscan, showplot=False, saveplot=cluster_filepath)
        #print("len of cluster_lst,", len(cluster_lst))




        line = str()

        per_floot_poly = []



        poly_count = 0
        for np_member_array in cluster_lst:
            poly_count += 1
            print(len(np_member_array))
            print("starting concave hull")
            hull = concaveHull(np_member_array, k=k, if_optimal=False)

            self.WriteConcave2WKT(hull,floor_name,poly_count)

            poly = Polygon(hull)


            print("polygon validation is: ", poly.is_valid, poly.area)


            # PlotHullAndPoints(hull, np_member_array)

            poly_filepath = "./result/Overlap/" + floor_name + "/polygon" + str(poly_count) + ".png"

            OBB_points = GetNumpyOBB(np_member_array, show_plot=False)
            OBB_poly = Polygon(OBB_points.tolist())
            print("OBB_poly area,", OBB_poly.area, " name,", floor_name,"---------------------------------------------------------------------")

            if not poly.is_valid:
                print("Try to repair validation:")
                new_poly = poly.buffer(0)
                line = line + "Repaired_" + str(new_poly.is_valid) + "_" + str(float("{:.2f}".format(new_poly.area)))
                print(new_poly.is_valid, new_poly.area)
                # print(new_poly)
                un_poly = ops.unary_union(new_poly)
                print(type(un_poly), un_poly.is_valid, "Union area,", un_poly.area)

                # if the poly is wrong, replace with OBB_poly
                if un_poly.area < (0.3 * OBB_poly.area):
                    un_poly = OBB_poly

                per_floot_poly.append(un_poly)

                if un_poly.geom_type == 'MultiPolygon':
                    # multipolygon
                    #if i != 1 and i != 0:
                    for geom in un_poly.geoms:
                        xs, ys = geom.exterior.xy
                        plt.plot(xs, ys, color="r")
                    # plt.show()
                    plt.savefig(poly_filepath)
                    plt.close()
                        # SavePloyAndPoints(un_poly.geoms[0],np_member_array,filepath=poly_filepath)

                elif un_poly.geom_type == 'Polygon':
                    # polygon
                    #if i != 1 or i != 0:
                        # PlotPolyAndPoints(un_poly, np_member_array)
                    SavePloyAndPoints(un_poly, np_member_array, filepath=poly_filepath)
                else:
                    print("Error polygon generation from concave hull failed!")
            else:

                line = line + "True, Floor Area" + str( float("{:.2f}".format(poly.area)))
                print("Polygon True, no need repair")
                #if i != 1 or i != 0:
                    # PlotPolyAndPoints(poly, np_member_array)
                SavePloyAndPoints(poly, np_member_array, filepath=poly_filepath)

                per_floot_poly.append(poly)
    #-----------------------------------------------------------------------------------------

        return per_floot_poly # [polygon] or [polygons]




    def OverlapOneFloor(self):

        if not self.floor_name_lst:
            return
        dialog = OneFloorOverlapDialog()
        if dialog.exec():
            floornum, outputFilepath, yamlFilepath = dialog.getInputs()
            floornum = int(floornum)
            print(floornum, type(floornum))
        else:
            return
        if floornum or floornum == 0:
            #self.canvas._display.Context.RemoveAll(True)
            if not self.canvas.floor_compound_shapes_lst[floornum]:
                msg = QtWidgets.QMessageBox()
                msg.setIcon(QtWidgets.QMessageBox.Critical)
                msg.setText("Current floor has no shapes or geometry,"+ self.floor_name_lst[floornum])
                msg.setWindowTitle("Shapes or Geometry Error")
                msg.setStandardButtons(QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel)
                msg.show()
                msg.exec_()
                return

            floor_name_lst = [self.floor_name_lst[floornum]]
            storey_poly_lst = []

            if self.base_polygon:

                current_floor_poly_lst = self.GetFloorPolygon(floornum,outputFilepath,yamlFilepath)
                storey_poly_lst.append(current_floor_poly_lst)

            else:

                base_poly_lst = self.GetFloorPolygon(self.base_floor_num,outputFilepath,yamlFilepath)
                self.base_polygon = base_poly_lst[0]
                current_floor_poly_lst= self.GetFloorPolygon(floornum,outputFilepath,yamlFilepath)
                storey_poly_lst.append(current_floor_poly_lst)


            GetStoreyOverlap(self.base_polygon,storey_poly_lst,floor_name_lst, outputFilepath)


    def WriteConcave2WKT(self,hull_lst,floor_name,poly_count):

        if not os.path.exists('./result/WKT'):
            os.makedirs('./result/WKT')

        new_lst = []
        for p in hull_lst:
            new_lst.append([float("{:.3f}".format(p[0] + self.georeference_x)),
                            float("{:.3f}".format(p[1] + self.georeference_y))])
        geo_poly = Polygon(new_lst)
        str_poly = str(geo_poly)

        f = open('./result/WKT/' + floor_name + '_'+str(poly_count)+'.txt', "w+")
        f.write("name|wkt\n")
        line_str = floor_name + '_' + str(poly_count) + "|" + str_poly + '\n'
        f.write(line_str)
        f.close()


        # for i in range(0, len(storey_poly_lst)):
        #     f = open('./result/WKT/'+floor_name_lst[i]+'.txt',"w+")
        #     f.write("name|wkt\n")
        #
        #     per_floor_poly = storey_poly_lst[i]
        #     count = 0
        #     for poly in per_floor_poly:  #can be 1 or 2 poly
        #         a = list(poly.exterior.coords)
        #         new_lst = []
        #         for p in a:
        #             new_lst.append([float("{:.3f}".format(p[0] + self.georeference_x)), float("{:.3f}".format(p[1] +self.georeference_y))])
        #         geo_poly =Polygon(new_lst)
        #         str_poly = str(geo_poly)
        #         line_str = floor_name_lst[i]+'_'+str(count)+ "|" + str_poly +'\n'
        #         f.write(line_str)
        #         count+=1
        #     f.close()

    def OverlapAll(self):

        if not self.floor_name_lst:
            return


        dialog = AllFloorOverlapDialog()
        if dialog.exec():
            outputFilepath, yamlFilepath = dialog.getInputs()
        else:
            return
        #self.canvas._display.Context.RemoveAll(True)
        storey_poly_lst = []
        new_floor_name_lst = []
        if self.base_polygon:
            for i in range(self.base_floor_num,len(self.floor_elements_lst)):
                if self.canvas.floor_compound_shapes_lst[i]:

                    floor_poly_lst = self.GetFloorPolygon(i, outputFilepath,yamlFilepath)
                    storey_poly_lst.append(floor_poly_lst)
                    new_floor_name_lst.append(self.floor_name_lst[i])
        else:

            base_poly_lst = self.GetFloorPolygon(self.base_floor_num, outputFilepath,yamlFilepath)
            self.base_polygon = base_poly_lst[0]

            for i in range(self.base_floor_num,len(self.floor_elements_lst)):

                if self.canvas.floor_compound_shapes_lst[i]:

                    floor_poly_lst = self.GetFloorPolygon(i, outputFilepath,yamlFilepath)
                    storey_poly_lst.append(floor_poly_lst)
                    new_floor_name_lst.append(self.floor_name_lst[i])

        # write concave into WKT
#        self.WriteConcave2WKT(storey_poly_lst, new_floor_name_lst)



        GetStoreyOverlap(self.base_polygon,storey_poly_lst,new_floor_name_lst,outputFilepath)



    def OverlapOneFloorOBB(self):

        if not self.floor_name_lst:
            return

        floor_num, okPressed = QtWidgets.QInputDialog.getInt(self.window, 'InputFloorNumber', 'FloorNumber=')
        if okPressed:
            # self.canvas._display.Context.RemoveAll(True)

            self.floornum = floor_num
        else:
            return

        if floor_num or floor_num == 0:
            #self.canvas._display.Context.RemoveAll(True)
            floor_name_lst = [self.floor_name_lst[floor_num]]
            storey_poly_lst = []

            if self.base_polygon:
                if self.canvas.floor_compound_shapes_lst[floor_num]:

                    current_poly, current_poly_lst, all_pt_lst = self.GetFloorOBBPoly_new(floor_num)
                    storey_poly_lst.append([current_poly])
                else:
                    msg = QtWidgets.QMessageBox()
                    msg.setIcon(QtWidgets.QMessageBox.Critical)
                    msg.setText("Current floor has no shapes or geometry," + self.floor_name_lst[floor_num])
                    msg.setWindowTitle("Shapes or Geometry Error")
                    msg.setStandardButtons(QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel)
                    msg.show()
                    msg.exec_()
                    return

            else:
                if self.canvas.floor_compound_shapes_lst[floor_num]:

                    base_poly, base_poly_lst, base_all_pt_lst = self.GetFloorOBBPoly_new(self.base_floor_num)
                    self.base_polygon = base_poly
                    current_poly, current_poly_lst, all_pt_lst = self.GetFloorOBBPoly_new(floor_num)

                    storey_poly_lst.append([current_poly])
                else:
                    msg = QtWidgets.QMessageBox()
                    msg.setIcon(QtWidgets.QMessageBox.Critical)
                    msg.setText("Current floor has no shapes or geometry," + self.floor_name_lst[floor_num])
                    msg.setWindowTitle("Shapes or Geometry Error")
                    msg.setStandardButtons(QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel)
                    msg.show()
                    msg.exec_()
                    return

            if not os.path.exists('./result/OBB_Overlap'):
                os.makedirs('./result/OBB_Overlap')

            outputFilepath = './result/OBB_Overlap/'+self.floor_name_lst[floor_num]+'.txt'
            GetStoreyOverlap(self.base_polygon,storey_poly_lst,floor_name_lst, outputFilepath)

            print("message")
            msg = QtWidgets.QMessageBox()
            msg.setIcon(QtWidgets.QMessageBox.Information)
            msg.setText("OBB overlap result saved in "+outputFilepath)
            msg.setWindowTitle("OBB Overlap Result")
            msg.setStandardButtons(QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel)
            msg.show()
            msg.exec_()





    def OverlapAllOBB(self):

        if not self.floor_name_lst:
            return



        storey_poly_lst = []
        new_floor_name_lst = []

        if self.base_polygon:
            for i in range(self.base_floor_num, len(self.floor_elements_lst)):
                if self.canvas.floor_compound_shapes_lst[i]:

                    floor_poly,floor_poly_lst,floor_all_pt_lst = self.GetFloorOBBPoly_new(i)
                    storey_poly_lst.append([floor_poly])
                    new_floor_name_lst.append(self.floor_name_lst[i])
        else:

            base_poly, base_poly_lst, base_all_pt_lst = self.GetFloorOBBPoly_new(self.base_floor_num)
            self.base_polygon = base_poly

            for i in range(self.base_floor_num, len(self.floor_elements_lst)):
                if self.canvas.floor_compound_shapes_lst[i]:

                    floor_poly, floor_poly_lst, floor_all_pt_lst = self.GetFloorOBBPoly_new(i)
                    storey_poly_lst.append([floor_poly])
                    new_floor_name_lst.append(self.floor_name_lst[i])

        if not os.path.exists('./result/OBB_Overlap'):
            os.makedirs('./result/OBB_Overlap')

        outputFilepath = './result/OBB_Overlap/OBB_OverlapAll.txt'

        GetStoreyOverlap(self.base_polygon, storey_poly_lst, new_floor_name_lst, outputFilepath)

        print("message")
        msg = QtWidgets.QMessageBox()
        msg.setIcon(QtWidgets.QMessageBox.Information)
        msg.setText("All floor OBB overlap result saved in " + outputFilepath)
        msg.setWindowTitle("All Floor OBB Overlap Result")
        msg.setStandardButtons(QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel)
        msg.show()
        msg.exec_()



    def setOverhangdir(self):

        dialog = CheckInput()
        if dialog.exec():
            self.overhang_left =  dialog.getInputs()
        else:
            return


        print(self.overhang_left)




    # def ptsReorder(self,pt_lst): #input [[x,y]]
    #     new_list = sorted(pt_lst , key=lambda k: [k[0], k[1]]) # x,y min to max
    #
    #     return [new_list[3],new_list[1],new_list[0],new_list[2]]
        # x=[]
        # y=[]
        # for pt in pt_lst:
        #     x.append(pt[0])
        #     y.append(pt[1])
        # pt_xmin = None
        # pt_xmax = None
        # pt_ymin = None
        # pt_ymax = None
        #
        # for pt in pt_lst:
        #     if pt[0] == max(x):
        #         pt_xmax = pt
        #         continue
        #     if pt[0] == min(x):
        #         pt_xmin = pt
        #         continue
        #     if pt[]





    # def GetFloorOBBPoly(self,i):
    #
    #     #self.canvas._display.Context.EraseAll(True)
    #
    #
    #     floor_name = self.floor_name_lst[i]
    #     print("current floor, ", floor_name)
    #     #shapes = CreateShape(floor_elements)
    #     compound_shapes = self.canvas.floor_compound_shapes_lst[i]
    #
    #     # get 3d oriented BBox of the shapes
    #
    #     pts = GetOrientedBoundingBoxShapeCompound(compound_shapes)
    #
    #     corners_top = pts[0:4]
    #
    #     pyocc_corners_list = []
    #     for pt in corners_top:
    #         pyocc_corners_list.append(
    #             [float("{:.3f}".format(pt.X() + self.georeference_x)), float("{:.3f}".format(pt.Y() +self.georeference_y))])
    #         print([pt.X(), pt.Y()])
    #
    #     # change the order of pt lst
    #     pyocc_corners_list = ptsReorder(pyocc_corners_list)
    #
    #     poly_corners = Polygon(pyocc_corners_list)
    #
    #     return poly_corners, pyocc_corners_list

    def GetFloorOBBPoly_new(self,i):

        #self.canvas._display.Context.EraseAll(True)


        floor_name = self.floor_name_lst[i]
        print("current floor, ", floor_name)
        #shapes = CreateShape(floor_elements)
        compound_shapes = self.canvas.floor_compound_shapes_lst[i]

        # get all pt_lst
        all_pt_lst = []
        exp = TopExp_Explorer(compound_shapes, OCC.Core.TopAbs.TopAbs_VERTEX)
        while exp.More():
            vertex = OCC.Core.TopoDS.topods_Vertex(exp.Current())
            pnt = OCC.Core.BRep.BRep_Tool_Pnt(vertex)
            all_pt_lst.append([float("{:.3f}".format(pnt.X())),float("{:.3f}".format(pnt.Y()))])
            exp.Next()

        # get 3d oriented BBox of the shapes

        pts = GetOrientedBoundingBoxShapeCompound(compound_shapes)

        Z_value = []
        for pt in pts:
            Z_value.append(pt.Z())
        z_max = max(Z_value)
        z_min = min(Z_value)
        z_mid = 0.5 * (z_max + z_min)
        pts_low = []
        pts_up = []
        for pt in pts:
            if pt.Z() < z_mid:
                pts_low.append(pt)
            else:
                pts_up.append(pt)

        corners_top = pts_up


        pyocc_corners_list = []
        for pt in corners_top:
            pyocc_corners_list.append(
                #[float("{:.3f}".format(pt.X() + self.georeference_x)), float("{:.3f}".format(pt.Y() +self.georeference_y))])
                [float("{:.3f}".format(pt.X())), float("{:.3f}".format(pt.Y() ))])
            #print([pt.X(), pt.Y()])

        # change the order of pt lst
        pyocc_corners_list = ptsReorder(pyocc_corners_list)

        poly_corners = Polygon(pyocc_corners_list)

        return poly_corners, pyocc_corners_list, all_pt_lst


    def OverhangAll_manual(self):


        pass

    def OverhangAll_new(self):

        if not self.floor_name_lst:
            return




        f=open("./result/overhang_all.txt", "w+")

        self.base_overhang_obb_poly, self.base_obb_pt_lst, base_all_pt_lst = self.GetFloorOBBPoly_new(self.base_floor_num)

        up_overhang_lst = []
        low_overhang_lst =[]

        for i in range(self.base_floor_num,len(self.floor_elements_lst)):
            current_floor_obb_poly, current_obb_pt_lst, current_all_pt_lst = self.GetFloorOBBPoly_new(i)
            up_overhang, low_overhang = self.OBBPolyOverhang_new(self.base_obb_pt_lst, current_all_pt_lst,self.overhang_left)
            up_overhang_lst.append(up_overhang)
            low_overhang_lst.append(low_overhang)

            #print("floor name, ", self.floor_name_lst[i], " up_overhang, ", up_overhang, "low_overhang, ", low_overhang)

        f.write("Overhang information ! \n")
        up_idx = up_overhang_lst.index(max(up_overhang_lst))
        low_idx = low_overhang_lst.index(max(low_overhang_lst))

        str1 = "max overhang of north direction, floor: " + self.floor_name_lst[up_idx + self.base_floor_num]+ ",  overhang distance: " + str(max(up_overhang_lst)) +" meter.\n"
        f.write(str1)
        str2 = "max overhang of south direction, floor: " + self.floor_name_lst[low_idx + self.base_floor_num] + ",  overhang distance: " + str(max(low_overhang_lst)) + " meter.\n"
        f.write(str2)



        for i in range(len(up_overhang_lst)):

            str_floor ="floor name: " + self.floor_name_lst[i + self.base_floor_num] +" north overhang "+  str(up_overhang_lst[i]) + " meter," +\
                       "south overhang " + str(low_overhang_lst[i]) + " meter \n"
            f.write(str_floor)
        f.close()

        msg = QtWidgets.QMessageBox()
        msg.setIcon(QtWidgets.QMessageBox.Information)
        msg.setText("All floor overhang done!\n  result save in result/overhang_all.txt")
        msg.setWindowTitle("All floor overhang")
        msg.setStandardButtons(QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel)
        msg.show()
        msg.exec_()



    def OverhangOneFloor(self):

        if not self.floor_name_lst:
            return

        floor_num, okPressed = QtWidgets.QInputDialog.getInt(self.window, 'Single Floor Overhang', 'FloorNumber=')
        if okPressed:
            # self.canvas._display.Context.RemoveAll(True)

            self.floornum = floor_num
        else:
            return

        if floor_num>=0 and floor_num < len(self.floor_elements_lst):
            #self.canvas._display.Context.RemoveAll(True)
            #floor_name_lst = [self.floor_name_lst[floor_num]]
            #storey_poly_lst = []

            if self.base_overhang_obb_poly:
                #current_floor_obb_poly, current_obb_pt_lst= self.GetFloorOBBPoly(floor_num)

                current_floor_obb_poly, current_obb_pt_lst, current_all_pt_lst= self.GetFloorOBBPoly_new(floor_num)
            else:
                #self.base_overhang_obb_poly, self.base_obb_pt_lst = self.GetFloorOBBPoly(1)
                #current_floor_obb_poly, current_obb_pt_lst= self.GetFloorOBBPoly(floor_num)

                self.base_overhang_obb_poly, self.base_obb_pt_lst, base_all_pt_lst = self.GetFloorOBBPoly_new(self.base_floor_num)
                current_floor_obb_poly, current_obb_pt_lst, current_all_pt_lst= self.GetFloorOBBPoly_new(floor_num)


            #up_overhang, low_overhang = self.OBBPolyOverhang(self.base_obb_pt_lst,current_obb_pt_lst)

            up_overhang, low_overhang = self.OBBPolyOverhang_new(self.base_obb_pt_lst,current_all_pt_lst,self.overhang_left)

            print("OverhangOneFloor done! --------------------------------------------------")
            print("floor name, ",self.floor_name_lst[floor_num], " up_overhang, ",up_overhang, "low_overhang, ", low_overhang)

        else:
            print("message")
            msg = QtWidgets.QMessageBox()
            msg.setIcon(QtWidgets.QMessageBox.Critical)
            msg.setText("Floor number is out of range")
            msg.setWindowTitle("Floor number Error")
            msg.setStandardButtons(QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel)
            msg.show()
            msg.exec_()

    def OBBPolyOverhang_new(self, base_pt_lst, target_pt_lst, left_side=True ):

        # upper line index 2, 3

        # p_up_0 = base_pt_lst[2]
        # p_up_1 = base_pt_lst[3]

        if left_side:
            p_up_0 = base_pt_lst[0]
            p_up_1 = base_pt_lst[3]

        # lower line index 1, 0
        # p_low_0 = base_pt_lst[1]
        # p_low_1 = base_pt_lst[0]
            p_low_0 = base_pt_lst[1]
            p_low_1 = base_pt_lst[2]

        else:
            p_up_0 = base_pt_lst[3]
            p_up_1 = base_pt_lst[0]

            p_low_0 = base_pt_lst[2]
            p_low_1 = base_pt_lst[1]


        # up distance:
        dis_up = [0.0]
        dis_low =[0.0]

        for t in target_pt_lst:
            v_up = (p_up_1[0] - p_up_0[0])*(t[1] - p_up_0[1]) - (t[0] - p_up_0[0])*(p_up_1[1]-p_up_0[1])
            if v_up>0:
                d = PT2lineDistance(p_up_0,p_up_1,t)
                dis_up.append(float("{:.3f}".format(d)))
                continue
            v_low = (p_low_1[0] - p_low_0[0])*(t[1] - p_low_0[1]) - (t[0] - p_low_0[0])*(p_low_1[1]-p_low_0[1])
            if v_low <0:
                d2 = PT2lineDistance(p_low_0,p_low_1,t)
                dis_low.append(float("{:.3f}".format(d2)))

        up_overhang = max(dis_up)
        low_overhang = max(dis_low)

        #return up_overhang, low_overhang

        if left_side:
            return up_overhang,low_overhang   #always return up side and low side
        else:
            return low_overhang,up_overhang

#     def OBBPolyOverhang(self, base_pt_lst, target_pt_lst ):
#
#         # upper line index 2, 3
#
#         p_up_0 = base_pt_lst[2]
#         p_up_1 = base_pt_lst[3]
#
#         # lower line index 1, 0
#         p_low_0 = base_pt_lst[1]
#         p_low_1 = base_pt_lst[0]
#
#         # up distance:
#         dis_v2 = 0
#         dis_v3 = 0
#
#         v2 = (p_up_1[0] - p_up_0[0])*(target_pt_lst[2][1] - p_up_0[1]) - (target_pt_lst[2][0] - p_up_0[0])*(p_up_1[1]-p_up_0[1])
#         if v2>0:
#             dis_v2 = PT2lineDistance(p_up_0,p_up_1,target_pt_lst[2])
#
#         v3 = (p_up_1[0] - p_up_0[0])*(target_pt_lst[3][1] - p_up_0[1]) - (target_pt_lst[3][0] - p_up_0[0])*(p_up_1[1]-p_up_0[1])
#         if v3>0:
#             dis_v3 = PT2lineDistance(p_up_0,p_up_1,target_pt_lst[3])
#
#         up_overhang = max(dis_v2,dis_v3)
#
#         # low distance:
#         dis_v0 = 0
#         dis_v1 = 0
#
#         v0 = (p_low_1[0] - p_low_0[0])*(target_pt_lst[0][1] - p_low_0[1]) - (target_pt_lst[0][0] - p_low_0[0])*(p_low_1[1]-p_low_0[1])
#         if v0 <0:
#             dis_v0 = PT2lineDistance(p_low_0,p_low_1,target_pt_lst[0])
#
#         v1 = (p_low_1[0] - p_low_0[0])*(target_pt_lst[1][1] - p_low_0[1]) - (target_pt_lst[1][0] - p_low_0[0])*(p_low_1[1]-p_low_0[1])
#         if v1<0:
#             dis_v1 = PT2lineDistance(p_low_0,p_low_1,target_pt_lst[1])
#
#         low_overhang = max(dis_v0,dis_v1)
#
#         return up_overhang,low_overhang
#
#
#
#     def OverhangAll(self):
#
#         dialog = AllFloorOverlapDialog()
#         dialog.setWindowTitle("Overhang all floors")
#         if dialog.exec():
#             outputFilepath, yamlFilepath = dialog.getInputs()
#         else:
#             return
#         f = open(outputFilepath, "w+")
#         for i in range(0, len(self.floor_elements_lst)):
#             floor_elements = self.floor_elements_lst[i]
#             floor_name = self.floor_name_lst[i]
#             print("current floor, ", floor_name)
#             compound_shapes = self.canvas.floor_compound_shapes_lst[i]
#
# #------------------------------------------------------
#             # # get 3d oriented BBox of the shapes
#             # # aBox, center_pt, [aHalfX, aHalfY, aHalfZ], corners_bot, corners_top = GetOrientedBoundingBox(floor_elements)
#             # pts = GetOrientedBoundingBoxShapeCompound(compound_shapes)
#             #
#             # Z_value = []
#             # for pt in pts:
#             #     Z_value.append(pt.Z())
#             #
#             # z_max = max(Z_value)
#             # z_min = min(Z_value)
#             #
#             #
#             # print("bot,top, z values,", z_min, z_max)
# # -----------------------------------------------------------
#             # get 3d oriented BBox of the shapes
#             aBox, center_pt, [aHalfX, aHalfY, aHalfZ], corners_bot, corners_top = GetOrientedBoundingBoxShapeCompound_old(compound_shapes)
#
#
#             pyocc_corners_list = []
#             for pt in corners_top:
#                 pyocc_corners_list.append([pt.X(), pt.Y()])
#                 print([pt.X(), pt.Y()])
#             poly_corners = Polygon(pyocc_corners_list)
#             # plot pythonocc obb
#             # PlotPolyAndPoints(poly_corners,np.array(pyocc_corners_list))
#
#             x_max, x_min, y_max, y_min, z_max, z_min = GetCornerMaxMin(corners_bot, corners_top)
#
#             if corners_bot[0].Z() == corners_top[0].Z():
#                 print("bot,top, z values,", z_min, z_max)
#             else:
#                 print("2. bot,top, z values,", corners_bot[0].Z(), corners_top[0].Z())
#
# # ---------------------------------------------------------------
#
#             cutting_height = z_min + 1.5
#
#             # parameters
#             s = 0.5
#
#             if i == 1:
#                 cutting_height = z_max - 0.2
#
#             if not os.path.exists('./result/overhang/' + floor_name):
#                 os.makedirs('./result/overhang/' + floor_name)
#
#             # get the section shape
#             print("cutting height,", cutting_height)
#             section_shape = GetSectionShape(cutting_height, compound_shapes)
#
#             # get the section shape edges
#             edges = GetShapeEdges(section_shape)
#             print("len of total section edges,", len(edges))
#
#             first_xy = GetEdgeSamplePointsPerDistance(edges, s)
#             print("len of the points after sample,", len(first_xy))
#             np_points = np.array(first_xy)
#
#             f.write(floor_name + "\n")
#
#             calcconvexhull = False
#             # Get the oriented bounding box of the edges points
#             if i == 33:
#                 calcconvexhull = True
#
#             corners = GetNumpyOBB(np_points, calcconvexhull=calcconvexhull, show_plot=False)
#             before_floor_corner = corners
#             OBB_poly = Polygon(corners.tolist())
#             # ---------- first figure-------------------------------------#
#             file_path = "./result/overhang/" + floor_name + "/" + floor_name + ".png"
#             # PlotPolyAndPoints(OBB_poly,np_points,color='b')
#             SavePloyAndPoints(OBB_poly, np_points, color='b', filepath=file_path)
#             # Get the overhang distance information:
#             if i == 1 or i == 0:
#                 base_corners = GetNumpyOBB(np_points, show_plot=False)
#                 before_base_corner = base_corners
#                 pan_array = base_corners[3]
#                 base_corners = PanPoint(base_corners, pan_array)
#
#                 ground_poly = Polygon(corners.tolist())
#             pan_corner = PanPoint(corners, pan_array)
#             # print("Checking pan,", pan_corner)
#
#             new_corners = GetCoorAfterRotate(base_corners, pan_corner)
#
#             corner_poly = Polygon(new_corners)
#
#             if i == 1 or i == 0:
#                 base_poly = corner_poly
#             if i != 1:
#                 poly_file_path = "./result/overhang/" + floor_name + "/Polys.png"
#                 # --------------- second and third figures ---------------
#                 # Plot2Polys(Polygon(before_base_corner),Polygon(before_floor_corner))
#                 # Plot2Polys(base_poly,corner_poly)
#                 Save2Polys(base_poly, corner_poly, filepath=poly_file_path)
#
#             # print("new corners, after rotation,",new_corners)
#
#             min_y, max_y = GetArrayYminmax(np.array(new_corners))
#             if i == 1 or i == 0:
#                 base_min_y = min_y
#                 base_max_y = max_y
#
#             # print(new_corners)
#             f.write("Overhang distance Boompjes side," + str(float("{:.2f}".format(abs(base_max_y - max_y)))) +
#                     ",Hertekade side," + str(float("{:.2f}".format(abs(base_min_y - min_y)))) +
#                     "  Overhang information, min_y," + str(float("{:.2f}".format(min_y))) +
#                     ",max_y," + str(float("{:.2f}".format(max_y))) + "\n")
#
#             print("Overhang distance Boompjes side," + str(float("{:.2f}".format(abs(base_max_y - max_y)))) +
#                   ",Hertekade side," + str(float("{:.2f}".format(abs(base_min_y - min_y)))) +
#                   "Overhang information, min_y," + str(float("{:.2f}".format(min_y))) + ",max_y," + str(float("{:.2f}".format(max_y))) +
#                   "\n")
#         f.close()








    def addGeoreferencePoint(self):
        dialog = GeoreferencePTInputDialog()
        if dialog.exec():
            x,y,z =  dialog.getInputs()
            if x:
                self.georeference_x = float(x)
                self.georeference_y = float(y)
                self.georeference_z = float(z)
        else:
            return



    def footprintWKT(self):
        if not self.floor_name_lst:
            return
        dialog = FootprintDialog()
        if dialog.exec():
            floornum,outputfile = dialog.getInputs()
            floornum = int(floornum)
        else:
            return

        if floornum >= 0 and floornum < len(self.floor_elements_lst):
            shapes_compound = self.canvas.floor_compound_shapes_lst[floornum]

            pts = GetOrientedBoundingBoxShapeCompound(shapes_compound, False)

            Z_value = []
            for pt in pts:
                Z_value.append(pt.Z())
            z_max = max(Z_value)
            z_min = min(Z_value)
            z_mid = 0.5 * (z_max + z_min)

            pts_low = []
            pts_up = []
            for pt in pts:
                if pt.Z() < z_mid:
                    pts_low.append(pt)
                else:
                    pts_up.append(pt)

            corners_top = pts_up

            pyocc_corners_list = []
            for pt in corners_top:
                pyocc_corners_list.append(
                    [float("{:.3f}".format(pt.X() + self.georeference_x)), float("{:.3f}".format(pt.Y() + self.georeference_y))])


            # convex hull pyocc_corners_list
            from scipy.spatial import ConvexHull
            points = np.array(pyocc_corners_list)
            hull = ConvexHull(points)
            result = []
            for idx in hull.vertices:
                result.append(pyocc_corners_list[idx])

            poly_footprint = Polygon(result)
            str_poly = str(poly_footprint)
            f = open(outputfile,"w+")
            f.write("name|wkt\n")
            line_str = self.floor_name_lst[floornum]+"|"+str_poly
            f.write(line_str)
            f.close()
        else:
            print("message")
            msg = QtWidgets.QMessageBox()
            msg.setIcon(QtWidgets.QMessageBox.Critical)
            msg.setText("Floor number is out of range")
            msg.setWindowTitle("Floor number Error")
            msg.setStandardButtons(QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel)
            msg.show()
            msg.exec_()

    def GetHeight(self):
        # Elevation property of ground floor is 0.0

        if not self.floor_name_lst:
            return
        top_shape_compound = self.canvas.floor_compound_shapes_lst[-1]

        z_lst = []
        exp = TopExp_Explorer(top_shape_compound, OCC.Core.TopAbs.TopAbs_VERTEX)
        while exp.More():
            vertex = OCC.Core.TopoDS.topods_Vertex(exp.Current())
            pnt = OCC.Core.BRep.BRep_Tool_Pnt(vertex)
            if float("{:.3f}".format(pnt.Z())) not in z_lst:
                z_lst.append( float("{:.3f}".format(pnt.Z())))
            exp.Next()
        str1 = "BIM height is " + str(max(z_lst)) + " meter"
        print("Max Z value is ", max(z_lst), " meter")
        msg = QtWidgets.QMessageBox()
        msg.setIcon(QtWidgets.QMessageBox.Information)
        msg.setText(str1)
        msg.setWindowTitle("BIM height")
        msg.setStandardButtons(QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel)
        msg.show()
        msg.exec_()




    def parkingCalcualate(self):

        dialog = ParkingInputDialog()
        if dialog.exec():
            inputfile,outputfile,zone =  dialog.getInputs()
        else:
            return
        print("Parking Units Calculation starts!")
        print(inputfile,outputfile)
        if inputfile:
            ifc_file = ifcopenshell.open(inputfile)
            ifcSpaces = ifc_file.by_type('ifcspace')
            file_parking = open(outputfile, "w+")

            count_40, count_40_65, count_65_85, count_85_120, count_120_plus = GetIfcSpaceType(ifcSpaces)
            minpp = GetMinParkingUnitNum(count_40, count_40_65, count_65_85, count_85_120, count_120_plus, zone.upper())
            str_apartment = "number of Apartments, \nless 40 square meter:" + str(
                count_40) + "\n40 to 65 square meter:" + str(count_40_65) + "\n65 to 85 square meter:" + str(
                count_65_85) + "\n85 to 120 square meter:" + str(count_85_120) + "\nmore than 120 square meter:" + str(
                count_120_plus) + "\n"
            str_zone = "Zone type: Zone A Metropolitan  area\n"
            str_minpp = "min parking units to provide:" + str(minpp)
            file_parking.write(str_apartment)
            file_parking.write(str_zone)
            file_parking.write(str_minpp)
            file_parking.close()

            #------------------message---------------------
            msg = QtWidgets.QMessageBox()
            msg.setIcon(QtWidgets.QMessageBox.Information)
            msg.setText("Parking Units Calculation Done\n"+"File saved in:" +outputfile)
            msg.setWindowTitle("Information")
            msg.setStandardButtons(QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel)
            msg.show()
            msg.exec_()

    def setOverlapParameters(self):

        dialog = OverlapParameterInputDialog()
        if dialog.exec():
            x, y, z = dialog.getInputs()
            print(x,y,z)
            if x:
                self.s = float(x)
                self.dbscan = float(y)
                self.k = float(z)
        else:
            return




# ----------------------------- UI ------------------------------------------------#
class AllFloorOverlapDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.label1 = QtWidgets.QLabel("Output file")
        self.labelyaml = QtWidgets.QLabel("Load yaml file")
        self.outputFilepath = QtWidgets.QLineEdit(self)
        self.yamlFilepath = QtWidgets.QLineEdit(self)


        buttonBox = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel, self);
        buttonOutputFile = QtWidgets.QPushButton("Save file", self)
        buttonYaml = QtWidgets.QPushButton("Load yaml file", self)

        buttonOutputFile.resize(20, 22)

        buttonOutputFile.clicked.connect(self.openOutput)
        buttonYaml.clicked.connect(self.openInput)

        layout = QtWidgets.QGridLayout(self)
        layout.addWidget(self.label1, 0, 0)
        layout.addWidget(self.outputFilepath, 0, 1)
        layout.addWidget(buttonOutputFile, 0, 2)

        layout.addWidget(self.labelyaml, 2, 0)
        layout.addWidget(self.yamlFilepath, 2, 1)
        layout.addWidget(buttonYaml, 2, 2)


        layout.addWidget(buttonBox, 3, 1)

        buttonBox.accepted.connect(self.accept)
        buttonBox.rejected.connect(self.reject)

        self.setWindowTitle("All Floor Overlap Dialog")
        #self.show()

    def openOutput(self):
        fileName= QtWidgets.QFileDialog.getSaveFileName(self, 'SaveFile',"Overlap.txt")
        print(fileName[0],type(fileName),fileName)
        self.outputFilepath.setText(fileName[0])
        print(fileName[0])

    def openInput(self):
        fileName = QtWidgets.QFileDialog.getOpenFileName(self, 'OpenFile', ".",'(*.yaml)')
        print(fileName[0], type(fileName), fileName)
        self.yamlFilepath.setText(fileName[0])
        print(fileName[0])

    def getInputs(self):
        return self.outputFilepath.text(), self.yamlFilepath.text()

class FootprintDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.label1 = QtWidgets.QLabel("Input Floor number")

        self.label2 = QtWidgets.QLabel("Output file")

        self.floorNumber = QtWidgets.QLineEdit(self)

        self.outputFilepath = QtWidgets.QLineEdit(self)

        buttonBox = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel, self);
        buttonOutputFile = QtWidgets.QPushButton("Save file", self)

        buttonOutputFile.resize(20, 22)
        buttonOutputFile.clicked.connect(self.openOutput)

        layout = QtWidgets.QGridLayout(self)
        layout.addWidget(self.label1, 0, 0)
        layout.addWidget(self.floorNumber, 0, 1)

        layout.addWidget(self.label2, 1, 0)
        layout.addWidget(self.outputFilepath, 1, 1)
        layout.addWidget(buttonOutputFile, 1, 2)

        layout.addWidget(buttonBox, 3, 1)

        buttonBox.accepted.connect(self.accept)
        buttonBox.rejected.connect(self.reject)

        self.setWindowTitle("Footprint WKT")


    def openOutput(self):
        fileName= QtWidgets.QFileDialog.getSaveFileName(self, 'SaveFile',"WKT.txt")
        print(fileName[0],type(fileName),fileName)
        self.outputFilepath.setText(fileName[0])
        print(fileName[0])

    def getInputs(self):
        return self.floorNumber.text(), self.outputFilepath.text()


class OneFloorOverlapDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.label1 = QtWidgets.QLabel("Input Floor number")
        self.labelyml = QtWidgets.QLabel("Load yaml file")
        self.label2 = QtWidgets.QLabel("Output file")

        self.floorNumber = QtWidgets.QLineEdit(self)
        self.yamlFilepath = QtWidgets.QLineEdit(self)
        self.outputFilepath = QtWidgets.QLineEdit(self)

        buttonBox = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel, self);
        buttonOutputFile = QtWidgets.QPushButton("Save file", self)
        buttonYaml = QtWidgets.QPushButton("Load yaml file", self)

        buttonOutputFile.resize(20, 22)
        buttonOutputFile.clicked.connect(self.openOutput)
        buttonYaml.clicked.connect(self.openInput)

        layout = QtWidgets.QGridLayout(self)
        layout.addWidget(self.label1, 0, 0)
        layout.addWidget(self.floorNumber, 0, 1)


        layout.addWidget(self.label2, 1, 0)
        layout.addWidget(self.outputFilepath, 1, 1)
        layout.addWidget(buttonOutputFile, 1, 2)

        layout.addWidget(self.labelyml, 2 ,0)
        layout.addWidget(self.yamlFilepath, 2 ,1)
        layout.addWidget(buttonYaml, 2, 2)




        layout.addWidget(buttonBox, 3, 1)

        buttonBox.accepted.connect(self.accept)
        buttonBox.rejected.connect(self.reject)

        self.setWindowTitle("One Floor Overlap Dialog")
        #self.show()

    def openOutput(self):
        fileName= QtWidgets.QFileDialog.getSaveFileName(self, 'SaveFile',"Overlap.txt")
        print(fileName[0],type(fileName),fileName)
        self.outputFilepath.setText(fileName[0])
        print(fileName[0])

    def openInput(self):
        fileName = QtWidgets.QFileDialog.getOpenFileName(self, 'OpenFile',".",'(*.yaml)')
        print(fileName[0], type(fileName), fileName)
        self.yamlFilepath.setText(fileName[0])
        print(fileName[0])

    def getInputs(self):
        return self.floorNumber.text(), self.outputFilepath.text(), self.yamlFilepath.text()

class ParkingInputDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.choice = 'A'
        self.choice_list = ['B', 'C']

        self.label1 = QtWidgets.QLabel("Input file")
        self.label2 = QtWidgets.QLabel("Output file")
        self.label3 = QtWidgets.QLabel("Zone type")


        self.first = QtWidgets.QLineEdit(self)
        self.second = QtWidgets.QLineEdit(self)
        self.combobox_1 = QtWidgets.QComboBox(self)

        buttonBox = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel, self);

        buttonInputFile = QtWidgets.QPushButton("open file",self)
        buttonInputFile.resize(20,22)
        # buttonOpenFile.move(20, 100)
        buttonInputFile.clicked.connect(self.openInput)

        buttonOutputFile = QtWidgets.QPushButton("Save file", self)
        buttonOutputFile.resize(20, 22)
        buttonOutputFile.clicked.connect(self.openOutput)




        layout = QtWidgets.QGridLayout(self)
        layout.addWidget(self.label1, 0,0)
        layout.addWidget(self.first, 0,1)
        layout.addWidget(buttonInputFile, 0, 2)
        layout.addWidget(self.label2,1,0)
        layout.addWidget(self.second,1,1)
        layout.addWidget(buttonOutputFile,1,2)
        layout.addWidget(self.label3 ,2,0)
        layout.addWidget(self.combobox_1 ,2,1)



        layout.addWidget(buttonBox,3,1)
        # layout.addRow("Second text", self.second)
        # layout.addRow("filepath text", self.myTextBox)

        self.combobox_1.addItem(self.choice)  # 4
        self.combobox_1.addItems(self.choice_list)



        buttonBox.accepted.connect(self.accept)
        buttonBox.rejected.connect(self.reject)

        self.setWindowTitle("Parking unit calculation")
        #self.show()

    def getInputs(self):
        return self.first.text(), self.second.text(),self.combobox_1.currentText()

    def openInput(self):
        fileName= QtWidgets.QFileDialog.getOpenFileName(self, 'OpenFile')
        print(fileName[0],type(fileName),fileName)
        self.first.setText(fileName[0])
        print(fileName[0])

    def openOutput(self):
        fileName= QtWidgets.QFileDialog.getSaveFileName(self, 'SaveFile',"Parking.txt")
        print(fileName[0],type(fileName),fileName)
        self.second.setText(fileName[0])
        print(fileName[0])

class OverlapParameterInputDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.label1 = QtWidgets.QLabel("s")
        self.label2 = QtWidgets.QLabel("dbscan")
        self.label3 = QtWidgets.QLabel("k")


        self.s = QtWidgets.QLineEdit(self)
        self.dbscan = QtWidgets.QLineEdit(self)
        self.k = QtWidgets.QLineEdit(self)

        buttonBox = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel, self)

        layout = QtWidgets.QGridLayout(self)
        layout.addWidget(self.label1, 0,0)
        layout.addWidget(self.s, 0,1)

        layout.addWidget(self.label2,1,0)
        layout.addWidget(self.dbscan,1,1)

        layout.addWidget(self.label3,2,0)
        layout.addWidget(self.k,2,1)

        layout.addWidget(buttonBox,3,1)
        # layout.addRow("Second text", self.second)
        # layout.addRow("filepath text", self.myTextBox)




        buttonBox.accepted.connect(self.accept)
        buttonBox.rejected.connect(self.reject)
        self.setWindowTitle("Overlap parameters")

    def getInputs(self):
        return float(self.s.text()), float(self.dbscan.text()), float(self.k.text())

class GeoreferencePTInputDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.label1 = QtWidgets.QLabel("X")
        self.label2 = QtWidgets.QLabel("Y")
        self.label3 = QtWidgets.QLabel("Z")


        self.x = QtWidgets.QLineEdit(self)
        self.z = QtWidgets.QLineEdit(self)
        self.y = QtWidgets.QLineEdit(self)

        buttonBox = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel, self)

        layout = QtWidgets.QGridLayout(self)
        layout.addWidget(self.label1, 0,0)
        layout.addWidget(self.x, 0,1)

        layout.addWidget(self.label2,1,0)
        layout.addWidget(self.y,1,1)

        layout.addWidget(self.label3,2,0)
        layout.addWidget(self.z,2,1)

        layout.addWidget(buttonBox,3,1)
        # layout.addRow("Second text", self.second)
        # layout.addRow("filepath text", self.myTextBox)




        buttonBox.accepted.connect(self.accept)
        buttonBox.rejected.connect(self.reject)

        self.setWindowTitle("Georeference Point InputDialog")

    def getInputs(self):
        return self.x.text(), self.y.text(), self.z.text()


class CheckInput(QtWidgets.QDialog):

    def __init__(self):
        super().__init__()

        self.cb = QtWidgets.QCheckBox('Left', self)
        self.cb.move(20, 20)

        self.cb.toggle()

        self.setGeometry(50,50,320,200)
        self.setWindowTitle("Overhang direction")

        buttonBox = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel, self)
        buttonBox.move(50,100)

        buttonBox.accepted.connect(self.accept)
        buttonBox.rejected.connect(self.reject)


    def getInputs(self):
        return self.cb.isChecked()


if __name__ == "__main__":
    logging.getLogger('matplotlib.font_manager').disabled = True
    application().start()
    print("here")
