#!/usr/bin/env python

#sudo apt-get install python3-shapely
#pip install python-scipy

import rospy
import numpy as np
import scipy as sp
import scipy.spatial
from scipy.spatial import ConvexHull
from shapely.geometry import Point
from shapely.geometry import Polygon
from geometry_msgs.msg import Point as pt
from enoga.msg import points


#Будет публиковать сообщения с финальными внутренними точками зоны наводнения
pub_surf_points=0

#TODO: Если захотим избежать бесконченых отправок пограничных точек сюда и отсюда внутренних точек, то можно завести флаг,
# который будет установлен в 0, если значение новых точек отличается от предыдущих, и в 1, если значение точек равно старым.
# Тогда не будут обрабатываться полученные пограничные точки, и не будут формироваться и отправляться новые внутренние точки.
#TODO: Ещё лучше хранить список последних внутренних точек и просто обновлять его. Т.е. если придут такие же точки, что и раньше,
# то значение новых внутренних не поменяется, иначе изменится

#TODO: сделать отправку точек лидеру, чтобы тот уже обрабатывал далее сам эти точки

#Строим ограничительный прямоугольник, исходя из границ многоугольника
def getBoundingBox(points):
	boundingBox=[]
	x_min=points[0][0]
	x_max=points[0][0]
	y_min=points[0][1]
	y_max=points[0][1]
	
	for point in points:
		if point[0]>x_max:
			x_max=point[0]
		elif point[0]<x_min:
			x_min=point[0]
		if point[1]>y_max:
			y_max=point[1]
		elif point[1]<y_min:
			y_min=point[1]
	boundingBox.append(x_min-1)
	boundingBox.append(x_max+1)
	boundingBox.append(y_min-1)
	boundingBox.append(y_max+1)
	return boundingBox

#Проверяем, что точки лежат в ограничительном прямоугольнике
def in_box(points, bounding_box):
	return np.logical_and(np.logical_and(bounding_box[0]<=points[:, 0],
										 points[:, 0] <= bounding_box[1]),
						  np.logical_and(bounding_box[2]<=points[:, 1],
						  				 points[:, 1]<=bounding_box[3]))

#Получаем точки описывающего выпуклого многоугольника
def getConvexHullPoints(points):
	points_np=np.array(points.copy())
	chull=ConvexHull(points_np)
	convexhull_points=[]
	
	for i in range(len(points_np[chull.vertices, 0].tolist())):
		convexhull_points.append([points_np[chull.vertices, 0].tolist()[i], points_np[chull.vertices, 1].tolist()[i]])
	return convexhull_points

#Разбиваем наш граничный прямоугольник, включающий ннеобходимый нам многоугольник, на локусы (строим диаграмму Вороного)
def voronoi(robots, bounding_box):
	i=in_box(robots, bounding_box)
	#eps=sys.float_info.epsilon
	points_center=robots[i, :]
	points_left=np.copy(points_center)
	points_left[:, 0]=bounding_box[0]-(points_left[:, 0]-bounding_box[0])
	points_right=np.copy(points_center)
	points_right[:, 0]=bounding_box[1]+(bounding_box[1]-points_right[:, 0])
	points_down=np.copy(points_center)
	points_down[:, 1]=bounding_box[2]-(points_down[:, 1]-bounding_box[2])
	points_up=np.copy(points_center)
	points_up[:, 1]=bounding_box[3]+ (bounding_box[3]-points_up[:, 1])
	points=np.append(points_center,
					 np.append(np.append(points_left,
					 					 points_right,
					 					 axis=0),
							   np.append(points_down,
							   			 points_up,
							   			 axis=0),
							   axis=0),
					axis=0)
	
	#Compute Voronoi, what about qhull_options???!!
	vor=sp.spatial.Voronoi(points, qhull_options='Qbb Qc Qx')
	#Filter regions
	regions=[]
	ind=np.arange(points.shape[0])
	ind=np.expand_dims(ind, axis=1)
	
	for region in vor.regions:
		flag=True
		for index in region:
			if index== -1:
				flag=False
				break
			else:
				x=vor.vertices[index, 0]
				y=vor.vertices[index, 1]
				if not(bounding_box[0]-1 <= x and x<=bounding_box[1]+1 and
					   bounding_box[2]-1<=y and y<=bounding_box[3]+1):
					flag=False
					break
		if region!=[] and flag:
			regions.append(region)
	vor.filtered_points=points_center
	vor.filtered_regions=regions
	return vor

#Получени центра фигуры по точкам
def getCenter(points):
	center=[]
	if len(points)==2:
		center.append([(points[0][0]+points[1][0])/2, (points[0][1]+points[1][1])/2])
	else:
		x=[p[0] for p in points]
		y=[p[1] for p in points]
		center.append([sum(x)/len(points), sum(y)/len(points)])
	return center

#Получение внутренних точек
def getVoronoiInsidePoints(r, index):
	vertices_test=[]
	res_centers=[]
	bounding_box=getBoundingBox(r.tolist())
	vor=voronoi(r, bounding_box)
	
	locus_list=[]
	locus_list1=[]
	res_locus=[]
	points1=r.tolist()
	points2=points1.copy()
	points1=np.array(points1+[points1[0]]) # Замыкаем многоугольник
	polygon=Polygon(r.tolist())
	
	for region in vor.filtered_regions:
		vertices=vor.vertices[region, :]
		vertices_test.append(vertices)
	
	for locus in vertices_test:
		locus_polygon=Polygon(locus.tolist())
		locus_tmp=locus.tolist()
		for point in points2:
			if locus_polygon.contains(Point(point))==True or locus_polygon.covers(Point(point))==True:
				locus_tmp.append(list(point))
		locus_list.append(locus_tmp)
	for locus in locus_list:
		locus_list1=locus.copy()
		for point in locus:
			if polygon.contains(Point(point))==False and polygon.covers(Point(point))==False:
				locus_list1.remove(point)
		
		locus_list2=locus_list1.copy()
		for point in locus_list2:
			if point in points2:
				points_tmp=[]
				index=points2.index(point)
				points_tmp.append(points2[index-1])
				points_tmp.append(point)
				points_tmp=getCenter(points_tmp)
				locus_list1.append([points_tmp[0][0], points_tmp[0][1]])
				points_tmp.clear()
				points_tmp.append(point)
				if points2.index(point)!=len(points2)-1:
					points_tmp.append(points2[index+1])
				else:
					points_tmp.append(points2[0])
				points_tmp=getCenter(points_tmp)
				locus_list1.append([points_tmp[0][0], points_tmp[0][1]])
				points_tmp.clear()
					
		res_locus.append(locus_list1)
	for locus in res_locus:
		res_centers.append(getCenter(locus))
	
	#print(res_centers)
	return res_centers


old_points=[]
#Обработчик сообщений с точками границ зоны наводнения
def callback(data):
	#rospy.loginfo("Callback works")
	global old_points
	pre_points=[]
	res_points=[]
	for element in data.points:
		point=[element.x, element.y]
		pre_points.append(point)
	
	vor_points=getVoronoiInsidePoints(np.array(getConvexHullPoints(pre_points)), 0)
	for point in vor_points:
		res_point=pt()
		res_point.x=point[0][0]
		res_point.y=point[0][1]
		res_point.z=0
		res_points.append(res_point)
	if len(old_points)==0 or(old_points[0].x!=res_points[0].x and old_points[0].y!=res_points[0].y): 
		print(res_points)
		old_points=res_points.copy()
		pub_surf_points.publish(res_points)

    #test_res1=getVoronoiInsidePoints(np.array(getConvexHullPoints([[0.0, 4.0], [5.0, 2], [8, -1], [6.54508497, -7.75528258], [2, -7], [-1.54508497, -5.75528258], [-4.04508497, -1.93892626], [-4, 3]])), 0)

if __name__=='__main__':
    try:
       rospy.init_node("vbrac_surf_node")
       #Здесь считывается массив точек границы (точки граничных БЛА)
       rospy.Subscriber("/vbrac_surf_input", points, callback)
       
       #Сюда уходит массив внутренних точек
       pub_surf_points=rospy.Publisher("/vbrac_surf_points", points, queue_size=1000) 
       rate=rospy.Rate(2)
       while not rospy.is_shutdown():
           rate.sleep()
    except:
       pass
