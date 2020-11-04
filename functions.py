import ifcopenshell
import re
import math
import ifcopenshell.geom
from collections import defaultdict
from collections import OrderedDict

# --------- numpy ------------
import numpy as np
from numpy.linalg import norm

# ---------- python-occ library ---------
from OCC.Core.TopExp import TopExp_Explorer
import OCC.Core.TopExp
import OCC.Core.TopAbs
import OCC.Core.TopoDS
import OCC.Core.BRep
from OCC.Core.BRep import BRep_Tool
import OCC.Core.Bnd
import OCC.Core.gp
import OCC.Core.GProp
from OCC.Core.BRepGProp import brepgprop_VolumeProperties ,brepgprop_SurfaceProperties
from OCC.Core.gp import (gp_Vec, gp_Pnt, gp_Trsf, gp_OX, gp_OY,gp_Pln,
                         gp_OZ, gp_XYZ, gp_Ax2, gp_Dir, gp_GTrsf, gp_Mat)
from OCC.Core.BRepBndLib import brepbndlib_Add
from OCC.Core.Bnd import Bnd_Box, Bnd_OBB
from OCC.Extend.TopologyUtils import list_of_shapes_to_compound
from OCC.Extend.ShapeFactory import get_oriented_boundingbox
from OCC.Core.BRepBndLib import brepbndlib_Add, brepbndlib_AddOptimal, brepbndlib_AddOBB
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakePrism
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeFace
from OCC.OCCUtils.Topology import *


# # -------------database -------------
# import psycopg2
# import os
# #HOST = os.environ['HOST']
# #PW = os.environ['PW']

# ---------------CGAL alpha ----------------
from CGAL.CGAL_Kernel import *
from CGAL.CGAL_Alpha_shape_2 import Alpha_shape_2
from CGAL.CGAL_Triangulation_2 import Delaunay_triangulation_2

#----------Shapely --------------------
from shapely.geometry import Polygon, Point
from shapely import geometry,ops

# ------------- matpolt ---------------
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

# -------------Scipy ------------------
from scipy.interpolate import interp1d
from scipy.spatial import ConvexHull,convex_hull_plot_2d
from scipy.spatial import distance
# -------------kd-Tree-----------------
from scipy.spatial import KDTree, Rectangle, distance_matrix, cKDTree
from scipy.spatial.ckdtree import cKDTreeNode
from scipy.spatial import minkowski_distance

#-----------------sklearn--------------
from sklearn.cluster import DBSCAN
from sklearn import metrics
from sklearn.datasets import make_blobs
from sklearn.preprocessing import StandardScaler

from itertools import cycle

# ------------ overlap calculation -----
import time
from concavehull3 import concaveHull, PlotHull,plotPoints,PlotHullAndPoints
import os

# ------------ yml parameters ----------
import yaml
from yaml import Loader, Dumper



def GetElementsByStorey(ifc_file):

    ''' Get all the building storey elements from the input BIM , return list of storey elements, list of storey name'''

    # -----------------initialization --------------------------#
    building_elements_origin = ifc_file.by_type('IfcBuildingElement')
    building_elements = []
    for e in building_elements_origin:
        if e.id() == 1175346:
            print("Find that target")
        else:
            building_elements.append(e)
    storey_dict = {}
    building_storeys = ifc_file.by_type('IfcBuildingStorey')
    storey_id_lst = []
    for storey in building_storeys:
        storey_id_lst.append(storey.id())
    for element in building_elements:
        storey_dict[element.id()] = "null"
    for element in building_elements:
        if element.ContainedInStructure:
            rel = element.ContainedInStructure
            floor = rel[0]  # need to get all the elements in that floor
            # ------------------get storey number ---------------------#
            str_list = str(floor).split(',')
            floor_num = (str_list[-1])[1:-1]
            # ----------if constrain is the storey constrain---------
            if int(floor_num) in storey_id_lst:
                floor_split = re.split('[()]', str(floor))
                element_str = floor_split[2].split(',#')
                # ----------update mapping dictionary: element: storey_Number ----------------------
                for i in range(len(element_str)):
                    if i == 0:
                        element_id = int(str(element_str[i])[1:])
                        storey_dict[element_id] = int(floor_num)

                    else:
                        storey_dict[int(element_str[i])] = int(floor_num)

    # --------------dictionary of list: storey number: [      ] --------------------#
    dict_storeys = defaultdict(list)
    element_num = 0
    for element in building_elements:
        element_num += 1
        id = element.id()
        if storey_dict[id] != "null":
            # print("Storey number:",storey_dict[id])
            dict_storeys[storey_dict[id]].append(element)
    storey_element_num = 0
    for each_floor in dict_storeys:
        storey_element_num += len(dict_storeys[each_floor])

    # ---------------trying sort dictionary ---------------#
    lst_storeys = list()
    dict_storeys_sorted = OrderedDict(sorted(dict_storeys.items()))
    lst_storey_name = list()
    for q in dict_storeys_sorted:
        lst_storey_name.append(ifc_file[q].Name)
        lst_storeys.append(dict_storeys_sorted[q])
    print("All available storey from ", lst_storey_name[0], "to ", lst_storey_name[-1])
    print("Total number of storey:", len(lst_storey_name))

    return lst_storeys, lst_storey_name

def GetStoreyElements(ifc_file,storey_number):

    ''' Get all the building storey elements of the input floor, return list of storey elements, list of storey name'''

    building_elements_origin = ifc_file.by_type('IfcBuildingElement')
    print("len of building_elements_origin, ",len(building_elements_origin))
    building_elements = []
    for e in building_elements_origin:
        if e.id() == 1175346:
            print("Find that target")
        else:
            building_elements.append(e)
    storey_dict = {}
    building_storeys = ifc_file.by_type('IfcBuildingStorey')
    storey_id_lst = []

    for storey in building_storeys:
        #print(storey.Name, str(storey.id()))
        storey_id_lst.append(storey.id())

    for element in building_elements:
        storey_dict[element.id()] = "null"
    for element in building_elements:
        if element.ContainedInStructure:
            rel = element.ContainedInStructure
            floor = rel[0]  # need to get all the elements in that floor
            #------------------get storey number ---------------------#
            str_list = str(floor).split(',')
            floor_num = (str_list[-1])[1:-1]
            # ----------if constrain is the storey constrain---------
            if int(floor_num) in storey_id_lst:
                floor_split = re.split('[()]', str(floor))
                element_str = floor_split[2].split(',#')
                # ----------update mapping dictionary: element: storey_Number ----------------------
                for i in range(len(element_str)):
                    if i == 0:
                        element_id = int(str(element_str[i])[1:])
                        storey_dict[element_id] = int(floor_num)
                    else:
                        storey_dict[int(element_str[i])] = int(floor_num)
     #--------------dictionary of list: storey number: [      ] --------------------#
    dict_storeys = defaultdict(list)
    element_num = 0
    for element in building_elements:
        element_num+=1
        id = element.id()
        if storey_dict[id] != "null":
            #print("Storey number:",storey_dict[id])
            dict_storeys[storey_dict[id]].append(element)
    storey_element_num = 0
    for each_floor in dict_storeys:
        storey_element_num += len(dict_storeys[each_floor])
    #---------------trying sort dictionary ---------------#
    lst_storeys =list()
    dict_storeys_sorted =  OrderedDict(sorted(dict_storeys.items()))
    lst_storey_name = list()
    for q in dict_storeys_sorted:
        lst_storey_name.append(ifc_file[q].Name)
        lst_storeys.append(dict_storeys_sorted[q])
    print("All available storey from ", lst_storey_name[0], "to ",lst_storey_name[-1])
    print("Total number of storey:",len(lst_storey_name))
    print("Current Storey:", lst_storey_name[storey_number] )

    return lst_storeys[storey_number] ,lst_storey_name[storey_number]


def CreateShape(ifc_elements):

    '''create geometry shapes from the input ifc buidling storey elements, return list of shapes'''

    settings = ifcopenshell.geom.settings()
    settings.set(settings.USE_PYTHON_OPENCASCADE, True)
    shapes =[]
    if isinstance(ifc_elements, list):
        for element in ifc_elements:
            if element.Representation:
                shape = ifcopenshell.geom.create_shape(settings, element).geometry
                shapes.append(shape)
    else:
        if ifc_elements.Representation:
            shape = ifcopenshell.geom.create_shape(settings, ifc_elements)
            shapes.append(shape)
    return shapes

def GetShapesOBB(shapes):

    ''' Get the oriented bounding box of the input shape'''

    shapes_compound, if_all_compound = list_of_shapes_to_compound(shapes)
    aBaryCenter, [aHalfX, aHalfY, aHalfZ], aBox = get_oriented_boundingbox(shapes_compound, 1)
    print("box center point, X Y Z size", aBaryCenter.X(), aBaryCenter.Y(), aBaryCenter.Z(), aHalfX, aHalfY, aHalfZ)
    corner_bot, corner_top = get_oriented_boundingbox_coor(shapes_compound, 1)
    center_pt = OCC.Core.gp.gp_Pnt(aBaryCenter)
    return aBox, center_pt, [aHalfX, aHalfY,aHalfZ], corner_bot, corner_top  # type(aBox)) #class 'OCC.Core.TopoDS.TopoDS_Solid', gp_Pnt, [float,float,float]

def GetOrientedBoundingBoxShapes(shapes, optimal_OBB=False):
    shapes_compound, if_all_compound = list_of_shapes_to_compound(shapes)

    obb = Bnd_OBB()
    if optimal_OBB:
        is_triangulationUsed = True
        is_optimal = True
        is_shapeToleranceUsed = False
        brepbndlib_AddOBB(shapes_compound, obb, is_triangulationUsed, is_optimal, is_shapeToleranceUsed)
    else:
        brepbndlib_AddOBB(shapes_compound, obb)
    aBaryCenter = obb.Center()

    aBaryCenter = obb.Center()
    aXDir = obb.XDirection()
    aYDir = obb.YDirection()
    aZDir = obb.ZDirection()
    aHalfX = obb.XHSize()
    aHalfY = obb.YHSize()
    aHalfZ = obb.ZHSize()

    ax = gp_XYZ(aXDir.X(), aXDir.Y(), aXDir.Z())
    ay = gp_XYZ(aYDir.X(), aYDir.Y(), aYDir.Z())
    az = gp_XYZ(aZDir.X(), aZDir.Y(), aZDir.Z())
    p = gp_Pnt(aBaryCenter.X(), aBaryCenter.Y(), aBaryCenter.Z())

    pt=[]
    pt.append(gp_Pnt(p.XYZ() - ax * aHalfX - ay * aHalfY - az * aHalfZ))
    pt.append(gp_Pnt(p.XYZ() + ax * aHalfX - ay * aHalfY - az * aHalfZ))
    pt.append(gp_Pnt(p.XYZ() + ax * aHalfX + ay * aHalfY - az * aHalfZ))
    pt.append(gp_Pnt(p.XYZ() - ax * aHalfX + ay * aHalfY - az * aHalfZ))
    pt.append(gp_Pnt(p.XYZ() - ax * aHalfX - ay * aHalfY - az * aHalfZ))
    pt.append(gp_Pnt(p.XYZ() + ax * aHalfX - ay * aHalfY - az * aHalfZ))
    pt.append(gp_Pnt(p.XYZ() + ax * aHalfX + ay * aHalfY - az * aHalfZ))
    pt.append(gp_Pnt(p.XYZ() - ax * aHalfX + ay * aHalfY - az * aHalfZ))

    return  pt


def PT2lineDistance(p1,p2,p3): # get the distance from P3 perpendicular to a line drawn between P1 and P2 [x,y]
    p1 = np.array(p1)
    p2 = np.array(p2)
    p3 = np.array(p3)

    d = norm(np.cross(p2 - p1, p1 - p3)) / norm(p2 - p1)
    return d

def ptsReorder(pt_lst):
    new_list = sorted(pt_lst , key=lambda k: [k[0], k[1]]) # order by x first min to max, when x = then y min to max
    return [new_list[0],new_list[1],new_list[3],new_list[2]]

def GetOrientedBoundingBoxShapeCompound(shapes_compound, optimal_OBB=False):

    obb = Bnd_OBB()
    if optimal_OBB:
        is_triangulationUsed = True
        is_optimal = True
        is_shapeToleranceUsed = False
        brepbndlib_AddOBB(shapes_compound, obb, is_triangulationUsed, is_optimal, is_shapeToleranceUsed)
    else:
        brepbndlib_AddOBB(shapes_compound, obb)
    aBaryCenter = obb.Center()
    aXDir = obb.XDirection()
    aYDir = obb.YDirection()
    aZDir = obb.ZDirection()
    aHalfX = obb.XHSize()
    aHalfY = obb.YHSize()
    aHalfZ = obb.ZHSize()

    ax = gp_XYZ(aXDir.X(), aXDir.Y(), aXDir.Z())
    ay = gp_XYZ(aYDir.X(), aYDir.Y(), aYDir.Z())
    az = gp_XYZ(aZDir.X(), aZDir.Y(), aZDir.Z())
    p = gp_Pnt(aBaryCenter.X(), aBaryCenter.Y(), aBaryCenter.Z())

    pt=[]
    pt.append(gp_Pnt(p.XYZ() - ax * aHalfX - ay * aHalfY - az * aHalfZ))
    pt.append(gp_Pnt(p.XYZ() + ax * aHalfX - ay * aHalfY - az * aHalfZ))
    pt.append(gp_Pnt(p.XYZ() + ax * aHalfX + ay * aHalfY - az * aHalfZ))
    pt.append(gp_Pnt(p.XYZ() - ax * aHalfX + ay * aHalfY - az * aHalfZ))
    pt.append(gp_Pnt(p.XYZ() - ax * aHalfX - ay * aHalfY + az * aHalfZ))
    pt.append(gp_Pnt(p.XYZ() + ax * aHalfX - ay * aHalfY + az * aHalfZ))
    pt.append(gp_Pnt(p.XYZ() + ax * aHalfX + ay * aHalfY + az * aHalfZ))
    pt.append(gp_Pnt(p.XYZ() - ax * aHalfX + ay * aHalfY + az * aHalfZ))

    return  pt

def GetOrientedBoundingBoxShapeCompound_old(shapes_compound):
    aBaryCenter, [aHalfX, aHalfY, aHalfZ], aBox = get_oriented_boundingbox(shapes_compound, 1)
    print("box center point, X Y Z size", aBaryCenter.X(), aBaryCenter.Y(), aBaryCenter.Z(), aHalfX, aHalfY, aHalfZ)
    corner_bot, corner_top = get_oriented_boundingbox_coor(shapes_compound, 1)

    center_pt = OCC.Core.gp.gp_Pnt(aBaryCenter)
    return aBox, center_pt, [aHalfX, aHalfY,
                             aHalfZ], corner_bot, corner_top  # type(aBox)) #class 'OCC.Core.TopoDS.TopoDS_Solid', gp_Pnt, [float,float,float]

def GetOrientedBoundingBox(ifc_elements):
    shapes = CreateShape(ifc_elements)
    shapes_compound, if_all_compound = list_of_shapes_to_compound(shapes)
    aBaryCenter, [aHalfX, aHalfY, aHalfZ], aBox = get_oriented_boundingbox(shapes_compound, 1)
    print("box center point, X Y Z size",aBaryCenter.X(), aBaryCenter.Y(), aBaryCenter.Z(), aHalfX, aHalfY, aHalfZ)
    corner_bot,corner_top = get_oriented_boundingbox_coor(shapes_compound,1)

    center_pt = OCC.Core.gp.gp_Pnt(aBaryCenter)
    return aBox, center_pt, [aHalfX, aHalfY, aHalfZ], corner_bot, corner_top #type(aBox)) #class 'OCC.Core.TopoDS.TopoDS_Solid', gp_Pnt, [float,float,float]

def get_oriented_boundingbox_coor(shape, optimal_OBB=False):

    '''return the oriented bounding box of the TopoDS_Shape `shape`'''

    obb = Bnd_OBB()
    if optimal_OBB:
        is_triangulationUsed = True
        is_optimal = True
        is_shapeToleranceUsed = False
        brepbndlib_AddOBB(shape, obb, is_triangulationUsed, is_optimal, is_shapeToleranceUsed)
    else:
        brepbndlib_AddOBB(shape, obb)
    # converts the bounding box to a shape
    aBaryCenter = obb.Center()
    aXDir = obb.XDirection()
    aYDir = obb.YDirection()
    aZDir = obb.ZDirection()
    aHalfX = obb.XHSize()
    aHalfY = obb.YHSize()
    aHalfZ = obb.ZHSize()

    ax = gp_XYZ(aXDir.X(), aXDir.Y(), aXDir.Z())
    ay = gp_XYZ(aYDir.X(), aYDir.Y(), aYDir.Z())
    az = gp_XYZ(aZDir.X(), aZDir.Y(), aZDir.Z())
    p = gp_Pnt(aBaryCenter.X(), aBaryCenter.Y(), aBaryCenter.Z())
    anAxes = gp_Ax2(p, gp_Dir(aZDir), gp_Dir(aXDir))
    anAxes.SetLocation(gp_Pnt(p.XYZ() - ax*aHalfX - ay*aHalfY - az*aHalfZ))
    #corner points on the ground face of the box

    new_origin = gp_Pnt(p.XYZ() - ax*aHalfX - ay*aHalfY - az*aHalfZ)# left_down
    right_up = gp_Pnt(p.XYZ() + ax*aHalfX + ay*aHalfY - az*aHalfZ)
    right_down= gp_Pnt(p.XYZ() + ax*aHalfX - ay*aHalfY - az*aHalfZ)
    left_up = gp_Pnt(p.XYZ() - ax*aHalfX + ay*aHalfY - az*aHalfZ)
    corners_bot = [new_origin, left_up,right_up,right_down]

    new_origin_top = gp_Pnt(p.XYZ() - ax * aHalfX - ay * aHalfY + az * aHalfZ)
    right_up_top = gp_Pnt(p.XYZ() + ax * aHalfX + ay * aHalfY + az * aHalfZ)
    right_down_top = gp_Pnt(p.XYZ() + ax * aHalfX - ay * aHalfY + az * aHalfZ)
    left_up_top= gp_Pnt(p.XYZ() - ax * aHalfX + ay * aHalfY + az * aHalfZ)
    corners_top = [new_origin_top,left_up_top,right_up_top,right_down_top]

    if corners_top[0].Z()>=corners_bot[0].Z(): #always return bot , top
        return corners_bot,corners_top
    else:
        return corners_top, corners_bot

def GetCornerMaxMin(corners_bot,corners_top):
    lst_x = [corners_top[0].X(),corners_top[1].X(),corners_top[2].X(),corners_top[3].X(),corners_bot[0].X(),corners_bot[1].X(),corners_bot[2].X(),corners_bot[3].X()]
    lst_y = [corners_top[0].Y(),corners_top[1].Y(),corners_top[2].Y(),corners_top[3].Y(),corners_bot[0].Y(),corners_bot[1].Y(),corners_bot[2].Y(),corners_bot[3].Y()]
    lst_z = [corners_top[0].Z(),corners_top[1].Z(),corners_top[2].Z(),corners_top[3].Z(),corners_bot[0].Z(),corners_bot[1].Z(),corners_bot[2].Z(),corners_bot[3].Z()]

    x_max = max(lst_x)
    x_min = min(lst_x)

    y_max = max(lst_y)
    y_min = min(lst_y)

    z_max = max(lst_z)
    z_min = min(lst_z)

    return x_max,x_min,y_max,y_min,z_max,z_min

def GetAllCoordinates(ifc_elements):
    settings = ifcopenshell.geom.settings()
    settings.set(settings.USE_PYTHON_OPENCASCADE, True)
    settings.set(settings.USE_WORLD_COORDS, True)
    lst_x = list()
    lst_y = list()
    lst_z = list()
    if isinstance(ifc_elements,list):
        print("load ifc elements list")
        for element in ifc_elements:
            if element.Representation:
                shape = ifcopenshell.geom.create_shape(settings, element)
                exp = OCC.Core.TopExp.TopExp_Explorer(shape.geometry, OCC.Core.TopAbs.TopAbs_VERTEX)
                while exp.More():
                    vertex = OCC.Core.TopoDS.topods_Vertex(exp.Current())
                    pnt = OCC.Core.BRep.BRep_Tool_Pnt(vertex)
                    lst_x.append(pnt.X())
                    lst_y.append(pnt.Y())
                    lst_z.append(pnt.Z())
                    #print(type(pnt.X()), type(pnt.Y()), type(pnt.Z()))
                    exp.Next()
            else:
                print(str(element.id()) + "No Rrepresentation")
    else:
        if ifc_elements.Representation:
            shape = ifcopenshell.geom.create_shape(settings, ifc_elements)
            exp = OCC.Core.TopExp.TopExp_Explorer(shape.geometry, OCC.Core.TopAbs.TopAbs_VERTEX)
            while exp.More():
                vertex = OCC.Core.TopoDS.topods_Vertex(exp.Current())
                pnt = OCC.Core.BRep.BRep_Tool_Pnt(vertex)
                lst_x.append(pnt.X())
                lst_y.append(pnt.Y())
                lst_z.append(pnt.Z())
                print(pnt.X(), pnt.Y(), pnt.Z())
                exp.Next()
        else:
            print(str(ifc_elements.id()) + "No Rrepresentation")

    return lst_x,lst_y,lst_z

def WriteCorners2file(corners_bot_list,corners_top_list,filepath):
    file = open(filepath,"w")
    for i in range(len(corners_bot_list)):
        str1 = str(i)+','+str(corners_bot_list[i][0].X())+','+str(corners_bot_list[i][0].Y())+','+str(corners_bot_list[i][0].Z())+','+str(corners_bot_list[i][1].X())+','+str(corners_bot_list[i][1].Y())+','+str(corners_bot_list[i][1].Z())+','+str(corners_bot_list[i][2].X())+','+str(corners_bot_list[i][2].Y())+','+str(corners_bot_list[i][2].Z())+','+str(corners_bot_list[i][3].X())+','+str(corners_bot_list[i][3].Y())+','+str(corners_bot_list[i][3].Z())+'\n'
        file.write(str1)
    file.close()

def GetSectionShape(z, shapes): # the mid z value of the storey, list of the storey shapes
    if isinstance(shapes, list):
        shapes_compound, if_all_compound = list_of_shapes_to_compound(shapes)

        plane = gp_Pln(gp_Pnt(0., 0., z), gp_Dir(0., 0., 1.))
        face = BRepBuilderAPI_MakeFace(plane).Shape()
        # Computes Shape/Plane intersection
        section = BRepAlgoAPI_Section(shapes_compound, face)
        section.Build()
        if section.IsDone():
            print("Successfully get the section shape")
            return section.Shape()
        else:
            print("ERROR, the section shape cannot be built")
    else:
        plane = gp_Pln(gp_Pnt(0., 0., z), gp_Dir(0., 0., 1.))
        face = BRepBuilderAPI_MakeFace(plane).Shape()
        # Computes Shape/Plane intersection
        section = BRepAlgoAPI_Section(shapes, face)
        section.Build()
        if section.IsDone():
            print("Successfully get the section shape")
            return section.Shape()

# --------------------CGAL ALPHA-----------------------
def Point_2_str(self):
    return "Point_2"+str((self.x(), self.y()))

def test_Alpha_shapes_2(lst_x,lst_y,value, show_plot=False, if_kd_tree = True):
    points = list()
    for i in range(len(lst_x)):
        points.append(Point_2(lst_x[i],lst_y[i]))
    a= Alpha_shape_2()
    a.make_alpha_shape(points)
    # mode = a.get_mode()
    # print(type(mode),mode)
    #a.set_mode(a.Mode.REGULARIZED) enumerate 0:GENERAL, 1:REGULARIZED
    a.set_mode(1)
    mode = a.get_mode()
    print(type(mode), mode)
    a.set_alpha(value)
    alpha_shape_edges = []
    alpha_shape_vertices = []
    #print("Checking edges")
    edge_classify =set()
    for it in a.alpha_shape_edges():
        edge_classify.add(a.classify(it))
        alpha_shape_edges.append(a.segment(it))
    print("Checking the edge classify type:")
    print(edge_classify)
    for it in a.alpha_shape_vertices():
        alpha_shape_vertices.append(it)
    if show_plot:
        showAlphaShape(points,alpha_shape_edges)
    poly1 = GetPolyfromAlpha(points,alpha_shape_edges,if_kd_tree)
    return poly1

def GetConnectedEdgeNew(lst_egdes):
    print("Edge connected without KD-Tree")
    result = []
    base = lst_egdes[0]
    result.append(base)
    lst_egdes.remove(base)
    while len(lst_egdes)>0:
        for source_target in lst_egdes:
            source_pt = source_target[0]
            target_pt = source_target[1]
            if base[1] == source_pt: # previous end to next begin connection
                result.append(source_target)
                lst_egdes.remove(source_target)
                base = source_target
                break
            if base[1] == target_pt:#previous end to next end connection
                result.append([target_pt,source_pt])
                lst_egdes.remove(source_target)
                base = [target_pt,source_pt]
                break

    return result

def GetConnectedEdgeKD(lst_egdes):#input list of list [[source_pt,target_pt]] output :list of list [[source_pt,target_pt]] with connected order
    source_pt_lst = []
    target_pt_lst = []
    source_pt_x = []
    source_pt_y = []
    dict = {}
    result_edges_lst = []
    for i in range(len(lst_egdes)) :
        source_pt= lst_egdes[i][0]
        target_pt = lst_egdes[i][1]
        source_pt_lst.append(source_pt)
        target_pt_lst.append(target_pt)
        dict[i] = 0 #mark all edges as unchecked
        source_pt_x.append(source_pt.x())
        source_pt_y.append(source_pt.y())
    #create KD tree base on source_pt
    a = np.array(source_pt_x)
    b = np.array(source_pt_y)
    points = np.column_stack((a, b))
    kd = cKDTree(points)

    base = 0 #first source_pt
    dict[base] = 1
    result_edges_lst.append(lst_egdes[0])
    print("checking while")
    flag = int(0)
    #while 0 in list(dict.values()):
    while flag<3:
        previous_dict_zero_count = list(dict.values()).count(0)
        kd_target  = [target_pt_lst[base].x(),target_pt_lst[base].y()]
        dis, idx = kd.query([kd_target], 1) #find the nearest point but itself
        #print("distance should be zero, ",dis.item(0))

        # new base
        base = idx.item(0)
        if lst_egdes[idx.item(0)] not in result_edges_lst:
            result_edges_lst.append(lst_egdes[idx.item(0)])
        dict[base] = 1
        if previous_dict_zero_count == list(dict.values()).count(0):
            flag+=1
        else:
            flag = 0
        #print(base)
        #print("0 count,",list(dict.values()).count(0))

    print("Edge reconnected! Unconnected edge number,",list(dict.values()).count(0) )
    return result_edges_lst


def GetPolyfromAlpha(points,alpha_shape_edges, if_kd_tree):
    lst_vertices = []
    lst_edges = []

    for i in range(len(alpha_shape_edges)): # order of edges is not correct, discontinuous
        seg = alpha_shape_edges[i]
        source_pt = seg.source()
        target_pt = seg.target()
        lst_vertices.append((source_pt.x(),source_pt.y()))
        lst_edges.append([source_pt,target_pt])

    if if_kd_tree:
        new_lst_edges = GetConnectedEdgeKD(lst_edges)
    else:
        new_lst_edges = GetConnectedEdgeNew(lst_edges)
    new_lst_vertices = []
    for source_target in new_lst_edges:

        source_pt = source_target[0]
        target_pt = source_target[1]
        new_lst_vertices.append((source_pt.x(),source_pt.y()))

    poly = Polygon(new_lst_vertices)
    print(new_lst_vertices)
    print("poly is valid,",poly.is_valid)
    return  poly


def showAlphaShape(points, alpha_shape_edges):
    x = [pt.x() for pt in points]
    y = [pt.y() for pt in points]
    plt.subplot(121)
    plt.scatter(x, y, s=1)

    # draw alpha shape
    lst_x = []
    lst_y = []
    plt.subplot(122)
    for i in range(len(alpha_shape_edges)):
        seg = alpha_shape_edges[i]
        source_pt = seg.source()
        target_pt = seg.target()

        plt.plot([source_pt.x(),target_pt.x()],[source_pt.y(),target_pt.y()],color = 'r')
    plt.show()

def GetMAXDisPointlistPoly(lst_pt, polygon):
    maxdis =0.0
    for pt in lst_pt:
        dis = polygon.distance(pt)
        if dis > maxdis:
            maxdix = dis
    print("maxdis",maxdis)
    return maxdis

def EdgeSplitNew(edges_ver_x,edges_ver_y,edges_idx):
    a = np.array(edges_ver_x)
    b = np.array(edges_ver_y)
    points = np.column_stack((a, b))
    tree = cKDTree(points)

    first_group_ed_idx = []
    first_group_ed_idx.append(edges_idx[0])
    target = [[edges_ver_x[0], edges_ver_y[0]]]
    for i in range(1,len(edges_ver_x)):

        dis, idx = tree.query(target, 1, distance_upper_bound= 2.0)
        first_group_ed_idx.append(edges_idx[idx])
        points = np.delete(points,idx)
        tree = cKDTree(points)
        target.append([edges_ver_x[idx],edges_ver_y[idx]])

def EdgeSplitShapely(lst_edges):

    lines = []
    for edge in lst_edges:
        t = Topo(edge)
        x_lst = []
        y_lst = []
        for vertex in t.vertices():
            pnt = BRep_Tool().Pnt(vertex)
            x_lst.append(pnt.X())
            y_lst.append(pnt.Y())

        line = geometry.LineString([[x_lst[0],y_lst[0]],[x_lst[1],y_lst[1]]])
        lines.append(line)
    multi_line = geometry.MultiLineString(lines)
    merge_line = ops.linemerge(multi_line)
    print(merge_line)

def GetShapeEdges(shape):
    #input opencad TopoDs_shape return lst of TopoDS_edges
    edges = []
    exp = TopExp_Explorer(shape, OCC.Core.TopAbs.TopAbs_EDGE)
    while exp.More():
        count = 0
        edges.append(exp.Current())
        exp.Next()
    return edges

def GetEdgeXY(edge):
    x = []
    y = []
    t= Topo(edge)
    for vertex in t.vertices():
        # print("vertices type",type(t.vertices()))  class 'list_iterator'
        pnt = BRep_Tool().Pnt(vertex)
        # print('X', pnt.X(), 'Y', pnt.Y(), 'Z', pnt.Z())
        x.append(pnt.X())
        y.append(pnt.Y())
    return x[0],y[0],x[1],y[1]

def GetEdgeSamplePointsPerDistance(edges, distance):
    lst_xy = []
    print("edges size," ,len(edges))
    for edge in edges:
        t = Topo(edge)
        source_target = []
        for vertex in t.vertices():
            pnt = BRep_Tool().Pnt(vertex)
            source_target.append(pnt)
        source =  source_target[0]
        target = source_target[1]
        x_new, y_new = SamplePoints(source,target,distance)
        for i in range(len(x_new)):
            if [x_new[i],y_new[i]] not in lst_xy:
                lst_xy.append([x_new[i],y_new[i]])

    return lst_xy

def SamplePoints(source, target, distance):
    x1 = source.X()
    y1 = source.Y()
    x2 = target.X()
    y2 = target.Y()
    a = np.array([x1,x2])
    b = np.array([y1,y2])
    f = interp1d(a, b, kind='linear')
    if x1!=x2:
        if x1>x2:
            distance=-distance
        if y1==y2:
            x_new = np.arange(x1, x2, distance)
            x_new = np.append(x_new, x2)
            y_new = np.array(y1)
            for i in range(len(x_new)-1):
                y_new=np.append(y_new,y1)
            return x_new,y_new

        x_new = np.arange(x1, x2, distance)
        x_new = np.append(x_new, x2)
        # print("x_new:",x_new)
        # print("a,",a)
        # print("b,",b)
        y_new = f(x_new)
        # print("y_new:",y_new)#errors hr
    else:
        if y1>y2:
            distance=-distance
        elif y1==y2:
            return a,b
        y_new = np.arange(y1,y2,distance)
        y_new = np.append(y_new,y2)
        x_new=np.array(x1)
        for i in range(len(y_new)-1):
            x_new = np.append(x_new,x1)
    return x_new,y_new

def GetEdges2DPT(edges):
    lst_xy = []
    for edge in edges:
        t = Topo(edge)
        for vertex in t.vertices():
            # print("vertices type",type(t.vertices()))  class 'list_iterator'
            pnt = BRep_Tool().Pnt(vertex)
            # print('X', pnt.X(), 'Y', pnt.Y(), 'Z', pnt.Z())
            if [pnt.X(),pnt.Y()] not in lst_xy:
                lst_xy.append([pnt.X(),pnt.Y()])

    return lst_xy

def ptdistance2D(pt1,pt2):
    x = pt1.X()-pt2.X()
    y = pt1.Y()-pt2.Y()
    #z =
    return math.sqrt(x*x+y*y)

def distanceXY(x1,y1,x2,y2):
    return math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))

def Edge2EdgeDistance(edge1,edge2):

    x0,y0,x1,y1 = GetEdgeXY(edge1)
    s0,q0,s1,q1 = GetEdgeXY(edge2)

    a = distanceXY(x0,y0,s0,q0)
    b = distanceXY(x0,y0,s1,q1)
    c = distanceXY(x1,y1,s0,q0)
    d = distanceXY(x1,y1,s1,q1)
    return min(a,b,c,d)

def AddData2np(lst1,lst2):
    a = np.array(lst1)
    b = np.array(lst2)
    points= np.column_stack((a,b))
    return points

def EdgeErrorRemove(edges,distance):
    print("EdgeErrorRemove")
    dict = {}
    lst_x = []
    lst_y = []
    outlier_edges = []
    for i in range(len(edges)):
        x0, y0, x1, y1 = GetEdgeXY(edges[i])
        #dict[(x0, y0, x1, y1)] = i
        dict[(x0,y0)] = i
        dict[(x1,y1)] = i
        lst_x.append(x0)
        lst_x.append(x1)
        lst_y.append(y0)
        lst_y.append(y1)
        dict[(x0, y0, x1, y1)] = i
    data = AddData2np(lst_x, lst_y)
    kd = cKDTree(data)
    for edge in edges:
        x0, y0, x1, y1 = GetEdgeXY(edge)

        target = [x0,y0]
        dis, idx = kd.query([target], 3)
        if dis.item(2)> distance:
            if edges[dict[(x0,y0)]] not in outlier_edges:
                outlier_edges.append(edges[dict[(x0,y0)]])
        target2 = [x1,y1]
        dis2,idx2 = kd.query([target2], 3)
        if dis2.item(2)>distance:
            if edges[dict[(x1, y1)]] not in outlier_edges:
                outlier_edges.append(edges[dict[(x1, y1)]])
    for ed in outlier_edges:
        edges.remove(ed)
    print("len of result edges,",len(edges),"len of outlier ed,",len(outlier_edges) )
    return edges, outlier_edges

def GetStoreyOverlap(ground_poly, storey_poly_lst,floor_name_lst,filepath):
    f =open(filepath,"w+")
    ground_area = ground_poly.area
    str1 = "ground polygon/base polygon,"+str(ground_poly.is_valid)+",area:"+str(float("{:.2f}".format(ground_poly.area)))+" square meter"
    f.write(str1 + "\n")
    for i in range(0,len(storey_poly_lst)):
        per_floor_poly = storey_poly_lst[i]
        sum=0
        for poly in per_floor_poly:
            print(type(poly),poly.is_valid,poly.area)
            a = ground_poly.intersection(poly)
            sum+=a.area
        #print(floor_name_lst[i],"overlap area,", sum,"overlap percentage:,",sum/ground_area )
        str2 = str(floor_name_lst[i])+",overlap area,"+str(float("{:.4f}".format(sum)))+" square meter,overlap percentage:"+str(float("{:.4f}".format(sum/ground_area*100.0)))
        print(str2)
        f.write(str2+"\n")
    f.close()

def GetConvexHullVertices(lst_x,lst_y, show_plt = True):
    a = np.array(lst_x)
    b = np.array(lst_y)
    points = np.column_stack((a, b))
    hull = ConvexHull(points)
    if show_plt:
        figure = convex_hull_plot_2d(hull)
        plt.suptitle("convexhull")
        plt.show()
    return hull.vertices # indices of the convex hull vertices

def GetDBSCANClusteringlst(np_data, eps=0.3, min_samples=10,showplot =True,saveplot=False):
    result = []
    db = DBSCAN(eps, min_samples).fit(np_data)
    core_samples = db.core_sample_indices_
    labels = db.labels_
    # Number of clusters in labels, ignoring noise if present.
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    print("Number of cluster,",n_clusters_)
    colors = cycle('bgrcmybgrcmybgrcmybgrcmy')
    for k, col in zip(set(labels), colors):
        if k == -1:
            # Black used for noise.
            col = 'k'
            markersize = 6
        class_members = [index[0] for index in np.argwhere(labels == k)]
        cluster_core_samples = [index for index in core_samples
                                if labels[index] == k]
        np_member = np_data[class_members]
        result.append(np_member)
        if showplot or saveplot:
            for index in class_members:
                x = np_data[index]
                if index in core_samples and k != -1:
                    markersize = 8
                else:
                    markersize = 4
                plt.plot(x[0], x[1], 'o', markerfacecolor=col,
                        markeredgecolor='k', markersize=markersize)
    if showplot:
        plt.title('Estimated number of clusters: %d' % n_clusters_)
        plt.show()
    if saveplot:
        plt.savefig(saveplot)
        plt.close()
    return result

def GetNumpyOBB(np_points,calcconvexhull=False, show_plot =True):
    ''' Get oriented bounding box
     input np.array points (N,2), return corners np array [5,2] repeated first points'''

    if calcconvexhull:
        _ch = ConvexHull(np_points)
        np_points = _ch.points[_ch.vertices]

    ca = np.cov(np_points, y=None, rowvar=0, bias=1)
    v, vect = np.linalg.eig(ca)
    tvect = np.transpose(vect)
    # use the inverse of the eigenvectors as a rotation matrix and
    # rotate the points so they align with the x and y axes
    ar = np.dot(np_points, np.linalg.inv(tvect))

    # get the minimum and maximum x and y
    mina = np.min(ar, axis=0)
    maxa = np.max(ar, axis=0)
    diff = (maxa - mina) * 0.5

    # the center is just half way between the min and max xy
    center = mina + diff
    # get the 4 corners by subtracting and adding half the bounding boxes height and width to the center
    corners = np.array([center + [-diff[0], -diff[1]], center + [diff[0], -diff[1]], center + [diff[0], diff[1]],
                        center + [-diff[0], diff[1]], center + [-diff[0], -diff[1]]])

    # use the the eigenvectors as a rotation matrix and
    # rotate the corners and the centerback
    corners = np.dot(corners, tvect)
    center = np.dot(center, tvect)
    if show_plot:
        fig = plt.figure(figsize=(12, 12))
        ax = fig.add_subplot(111)
        ax.scatter(np_points[:, 0], np_points[:, 1])
        ax.scatter([center[0]], [center[1]])
        ax.plot(corners[:, 0], corners[:, 1], '-')
        plt.axis('equal')
        plt.show()
    return  corners

def writeArray2Txt(filename,np_array):
    file = open(filename, "w")
    for i in range(len(np_array)):
        x = np_array[i].item(0)
        y = np_array[i].item(1)
        str1 = str(x)+","+str(y)+"\n"
        file.write(str1)
    file.close()

def Plot2Polys(poly1,poly2):
    x, y = poly1.exterior.xy
    plt.plot(x, y, color='r')

    x2,y2 = poly2.exterior.xy
    plt.plot(x2, y2, color='b')
    plt.axis('equal')
    plt.suptitle("overhang polys")
    plt.show()

def Save2Polys(poly1,poly2,filepath="./result/overhang/2Polys.png"):
    x, y = poly1.exterior.xy
    plt.plot(x, y, color='r')
    x2, y2 = poly2.exterior.xy
    plt.plot(x2, y2, color='b')
    plt.axis('equal')
    plt.suptitle("overhang polys")
    plt.savefig(filepath)
    plt.close()

def PlotPolyAndPoints(poly,dataset,color="r"):

    plt.plot(dataset[:, 0], dataset[:, 1], 'o', markersize=1, markerfacecolor='0.75',
             markeredgewidth=1)
    plt.axis('equal')
    plt.axis([min(dataset[:, 0]) - 10, max(dataset[:, 0]) + 10, min(dataset[:, 1]) - 10,
              max(dataset[:, 1]) + 10])

    x, y = poly.exterior.xy
    plt.plot(x, y, color=color)

    plt.suptitle("Polygon and points")
    plt.show()

def SavePloyAndPoints(poly,dataset,color="r",filepath="./result/images/PolyandPoints.png"):
    plt.plot(dataset[:, 0], dataset[:, 1], 'o', markersize=1, markerfacecolor='0.75',
             markeredgewidth=1)
    plt.axis('equal')
    plt.axis([min(dataset[:, 0]) - 10, max(dataset[:, 0]) + 10, min(dataset[:, 1]) - 10,
              max(dataset[:, 1]) + 10])

    x, y = poly.exterior.xy
    plt.plot(x, y, color=color)
    plt.savefig(filepath)
    plt.close()

def PanPoint(corners,np_ab):
    # pan corner point, with -a -b distance to x, y axises：
    return corners -np_ab

def GetNewCoordinates(Theta,x0,y0,a, b, lst_x,lst_y):
    # a,b pan distance
    # return list [[x,y],[x,y],...]
    new_xy_lst = []

    #counterclockwise
    for i in range(len(lst_x)):
        x_new = (lst_x[i]-x0)*math.cos(Theta) - (lst_y[i]-y0)*math.sin(Theta)+x0
        y_new = (lst_x[i]-x0)*math.sin(Theta) + (lst_y[i]-y0)*math.cos(Theta)+y0
        new_xy_lst.append([x_new,y_new])
    print("New coor:",new_xy_lst)
    return new_xy_lst

def GetArrayYminmax(np_array):
    #input np.array(N,2)
    array_y = np_array[:,1]

    min_y = array_y.min()
    max_y = array_y.max()
    return min_y,max_y

def GetIfcSpaceLabel(ifcspace_element):
    definition = ifcspace_element.IsDefinedBy[0]
    # To support IFC2X3, we need to filter our results.
    if definition.is_a('IfcRelDefinesByProperties'):
        property_set = definition.RelatingPropertyDefinition
        for property in property_set.HasProperties:
            if property.is_a('IfcPropertySingleValue'):
                return str(property.NominalValue.wrappedValue)
    else:
        raise Exception('An error occurred,IfcRelDefinesByProperties')

def Get_first_xy(floor_elements,s=0.2):
    shapes = CreateShape(floor_elements)
    print(type(shapes), type(shapes[0]))
    # get 3d oriented BBox of the shapes
    aBox, center_pt, [aHalfX, aHalfY, aHalfZ], corners_bot, corners_top = GetOrientedBoundingBox(floor_elements)
    pyocc_corners_list = []
    for pt in corners_top:
        pyocc_corners_list.append([pt.X(), pt.Y()])
        print([pt.X(), pt.Y()])
    poly_corners = Polygon(pyocc_corners_list)
    x_max, x_min, y_max, y_min, z_max, z_min = GetCornerMaxMin(corners_bot, corners_top)

    if corners_bot[0].Z() == corners_top[0].Z():
        print("bot,top, z values,", z_min, z_max)
    else:
        print("2. bot,top, z values,", corners_bot[0].Z(), corners_top[0].Z())

    # cutting height
    mid_z_value = (z_max + z_min) * 0.5
    quartile_z_value = z_min + (z_max - z_min) * 0.25
    cutting_height = z_min + 1.0  # 1.5 2.0 no balcony and using OBB

    # get the section shape
    print("cutting height,", cutting_height)
    section_shape = GetSectionShape(cutting_height, shapes)


    edges = GetShapeEdges(section_shape)
    print("len of total section edges,", len(edges))

    # sample points of input edges:
    first_xy = GetEdgeSamplePointsPerDistance(edges, s)
    print("len of the points after sample,", len(first_xy))
    return first_xy

#------------------------------------------parking---------------------------------#
def calc_volume(s):
    props = OCC.Core.GProp.GProp_GProps()
    brepgprop_VolumeProperties(s, props)
    return props.Mass()

def GetMidHeight(shape):
    bbox = Bnd_Box()
    brepbndlib_Add(shape, bbox, False)
    xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
    return (zmax+zmin)*0.5
def GetHeight(shape):
    bbox = Bnd_Box()
    brepbndlib_Add(shape, bbox, False)
    xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
    return zmax,zmin


def GetIfcSpaceArea(ifcspace):
    settings = ifcopenshell.geom.settings()
    settings.set(settings.USE_PYTHON_OPENCASCADE, True)
    shape = ifcopenshell.geom.create_shape(settings, ifcspace).geometry

    #print(type(shape))
    zmax,zmin = GetHeight(shape)
    z = zmax -zmin
    #print("cutting z :",z)
    #section_shape = GetSectionShape(z,shape)
    vol = calc_volume(shape)
    #print("volume,",vol)
    return vol/z

def CheckIfcSpaceArea(ifcspace):
    print("Checking real area.")
    settings = ifcopenshell.geom.settings()
    settings.set(settings.USE_PYTHON_OPENCASCADE, True)
    shape = ifcopenshell.geom.create_shape(settings, ifcspace).geometry
    zmax, zmin = GetHeight(shape)
    z_cut = zmin+0.5
    section_shape = GetSectionShape(z_cut, shape)
    edges = GetShapeEdges(section_shape)
    first_xy = GetEdgeSamplePointsPerDistance(edges, 0.2)
    np_points = np.array(first_xy)
    cluster_lst = GetDBSCANClusteringlst(np_points, 1.5)
    for np_member_array in cluster_lst:
        hull = concaveHull(np_member_array, k=16, if_optimal=False)
        PlotHull(hull)
        poly = Polygon(hull)
        print("Poly is_valid: ", poly.is_valid)
        print(poly.area)
    pass

def GetMinParkingUnitNum(count_40,count_40_65, count_65_85, count_85_120, count_120_plus, zone_type):
    if zone_type == 'A':
        num = 0.1*count_40 + 0.4*count_40_65 + 0.6*count_65_85 + 1.0* count_85_120 + 1.2*count_120_plus
        return num
    elif zone_type == 'B':
        num = 0.1 * count_40 + 0.5 * count_40_65 + 0.8 * count_65_85 + 1.0 * count_85_120 + 1.2 * count_120_plus
        return num
    elif zone_type =='C':
        num = 0.1 * count_40 + 0.6 * count_40_65 + 1.4 * count_65_85 + 1.6 * count_85_120 + 1.8 * count_120_plus
        return num
    else:
        print("please input zone_type: A OR B OR C!")
        return


def GetIfcSpaceID(IfcSpace):
    definition = IfcSpace.IsDefinedBy[0] #ifcSpace.IsDefinedBy <class 'tuple'> 4 (#201119=IfcRelDefinesByProperties
    str_split =str(definition).split('=')
    print("IfcSpace ID is: ",str_split[0])
    return str_split[0],int(str_split[0][1:])

def GetIfcSpaceType(ifcSpaces): # return num of type: count_40,count_40_65, count_65_85, count_85_120, count_120_plus,
    count_40 = 0
    count_40_65 = 0
    count_65_85 = 0
    count_85_120 = 0
    count_120_plus = 0
    for ifcSpace in ifcSpaces:
        label = GetIfcSpaceLabel(ifcSpace)
        if "TYPE A" in label.upper() or "TYPE B" in label.upper() or "TYPE C" in label.upper() or "TYPE D" in label.upper() or "TYPE E" in label.upper() or "TYPE F" in label.upper():
            #print(label)
            area = GetIfcSpaceArea(ifcSpace)
            if area < 40:
                count_40 +=1
            elif area >= 40 and area < 65:
                count_40_65+=1
            elif area >=65 and area <85:
                count_65_85 +=1
            elif area >=85 and area <120:
                count_85_120 +=1
            elif area >=120:
                count_120_plus+=1
    return count_40,count_40_65, count_65_85, count_85_120, count_120_plus