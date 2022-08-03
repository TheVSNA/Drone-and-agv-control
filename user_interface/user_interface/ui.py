from distutils.spawn import spawn
from turtle import distance
from mpl_toolkits.basemap import Basemap
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import io
import math
import json
import time

import rclpy
from rclpy.node import Node

from planner_interface.srv import MsgJson
from interfaces.msg import Mymsg, MsgRet

from PIL import Image
import requests

from matplotlib.widgets import Button


global target_list
target_list=[]

#position of the bottom-left corner 
origin_position_lat = 46.05044147070661
origin_position_lon = 11.16072442646732
origin_position_height = 548.0 #mslm

#position of spawn point
spawn_lat = 46.06875
spawn_lon = 11.17530556
spawn_height = 472.0    #mslm

research_area_x = 3500  #m
research_area_y = 3500  #m

##@class  MinimalClientAsync
# @brief This class is used to communicate with the planning service.
# This class is used to send the list of all the desidered target to the planning service so that it can calculate all the actions to exeute

class MinimalClientAsync(Node):
    ##@brief Function to initialize the class.
    # This function keep on trying to connect to its Service counterpart until the connection is established
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(MsgJson, 'plan')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = MsgJson.Request()

    ##Function that send through the service channel a json containing all the desidered destination.
    # The service then returns through the same channel a json containing all the action that the vehicle will have to perform.
    # @param[in] string A string containing a json with all the destinations.
    def send_request(self, string):
        self.req.in_json = string
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

##@class MinimalPublisher
# @brief Class used to send the data received with  MinimalClientAsync to robots_control through the "/ui" topic
class MinimalPublisher(Node):  

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Mymsg, 'ui', 10)
        
    ##Function that takes in input a json object (obtained by MinimalClientAsync), divide it in some arrays containing the action to perform, the drone that will perform the action and the coordinates
    #  and sends it to the robots_control package so that the vehicles can perform their missions.
    #  @param[in] json Json object containing all the actions to perform
    def send_data(self,json):
        msg = Mymsg()
        i=0
        print("Received these actions: ")
        for action in json:
            msg.action[i]=action["action"]
            msg.vehicle[i]=action["drone"]
            msg.x[i]=action["x"]
            msg.y[i]=action["y"]
            msg.z[i]=action["z"]
            print("action ",action["action"]," drone ",action["drone"]," x ",action["x"]," y ",action["y"]," z ",action["z"])
            i+=1
        print("\n\n")
        msg.length=i
        self.publisher_.publish(msg)

##@class MinimalSubscriber
# @brief Class that read the "/ret" topic in which robots_control will publish the final position of the vehicles so that the return path can be calculated
class MinimalSubscriber(Node):  #subscribe to recive positions of drones and rover to calculate return path (through planning service)

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(MsgRet, 'ret', self.callback,10)
    
    ##@brief Callback function used when robots_control publish information on the "/ret" topic
    # This function create a json containing the last position of all the drones and send it through MinimalClientAsync to the planning service.
    # When the service return the list of action to execute, it uses MinimalPublisher to send the information to robots_control.
    # @param[in] request A message containing the last position for each vehicle.
    def callback(self, request):
        y_rover = request.x_rover + distance_origin_spawn_y
        x_rover = request.y_rover + distance_origin_spawn_x

        y_drone_0 = request.x_drone_0 + distance_origin_spawn_y
        x_drone_0 = request.y_drone_0 + distance_origin_spawn_x
        z_drone_0 = request.z_drone_0 + spawn_height

        y_drone_1 = request.x_drone_1 + distance_origin_spawn_y
        x_drone_1 = request.y_drone_1 + distance_origin_spawn_x
        z_drone_1 = request.z_drone_1 + spawn_height

        retjson={
            "coordinates":{
                "lat": origin_position_lat, 
                "lon": origin_position_lon
            },
            "sizes":{
                "height": 3500,
                "width": 3500
            },
            "sensibility": 350,
            "pos_agv":{
                "x": x_rover,
                "y": y_rover
            },
            "drones":
            [
                {
                    "x": x_drone_0,
                    "y": y_drone_0,
                    "z": np.abs(z_drone_0)
                },
                {
                    "x": x_drone_1,
                    "y": y_drone_1,
                    "z": np.abs(z_drone_1)
                }
            ],
            "gui": "false"
        }
        temp = json.dumps(retjson)
        response = minimal_client.send_request(temp)

        temp = json.loads(response.out_json)["actions"]


        for i in range(len(temp)):
            temp[i]["x"] , temp[i]["y"] = temp[i]["y"] , temp[i]["x"]
            temp[i]["x"] -= distance_origin_spawn_y
            temp[i]["y"] -= distance_origin_spawn_x
            temp[i]["z"] -= spawn_height

        minimal_publisher.send_data(temp)


##@brief Function that calculate the distance in meters between two point (described by their latitude and longitude).
# @param[in] lat1 Latitude of the first point.
# @param[in] lat2 Latitude of the second point.
# @param[in] lon1 Longitude of the first point.
# @param[in] lon2 Longitude of the second point.
# @retval dx distance in meters between the two longitudes
# @retval dy distance in meters between the two latitudes
def xyDistance(lat1, lat2, lon1, lon2):

    circEq = 40075.017
    circPol = 40007.863
    dx = (lon2 - lon1) * math.cos((lat1+lat2) * math.pi/360) * circEq * 1000 / 360.0
    dy = (lat2 - lat1) * (circPol / 360) * 1000
    return [dx, dy] #, math.sqrt(dx*dx+dy*dy), math.atan2(dy, dx)*180/math.pi

##@brief Function called when the user click on the map.
# When the user click on the map to add a destination this function is triggered. If the clicked point is inside the research area the function calculate the latitude and longitude of the point, 
# it calculates the distance between the point and the starting position and saves it in an array.
def onclick(event):
    try:
        if(event.xdata<x_start or event.ydata<y_start or event.xdata>(x_start+3500) or event.ydata>(y_start+3500)): #verify if the position is inside the research area
            #print("Please choose a point inside the rectangle")
            return
        
        lon,lat = m(event.xdata,event.ydata, inverse =True)     #get lon lat coordinates of the point
        target_list.append((lon,lat))   #save point on the list

        print("Coordinates:",lon,lat)
        
        plt.scatter(event.xdata,event.ydata,alpha=0.9)  ##crate a pinpoint on the map
        plt.annotate("Lat:"+str(lat)+"\nLon:"+str(lon),(event.xdata,event.ydata))
        plt.pause(0.001)  

    except Exception as e: 
        print(e)

def deg2num(lat_deg, lon_deg, zoom):
  lat_rad = math.radians(lat_deg)
  n = 2.0 ** zoom
  xtile = int((lon_deg + 180.0) / 360.0 * n)
  ytile = int((1.0 - math.log(math.tan(lat_rad) + (1 / math.cos(lat_rad))) / math.pi) / 2.0 * n)
  return (xtile, ytile)

def num2deg(xtile, ytile, zoom):
  n = 2.0 ** zoom
  lon_deg = xtile / n * 360.0 - 180.0
  lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * ytile / n)))
  lat_deg = math.degrees(lat_rad)
  return (lat_deg, lon_deg)

##@brief Function used to collect and put together a siries of images reppresenting the map received from OpenStreetMap.
# @param[in] lat_deg Latitude of the north west corner of the map.
# @param[in] lon_deg Longitude of the north west conrner of the map.
# @param[in] delta_lat height of the map expressed in degrees.
# @param[in] delta_lon width of the map expressed in degrees.
# @param[in] zoom Level of detail of the images (the higher the level, the more images to be downloaded).
def getImageCluster(lat_deg, lon_deg, delta_lat,  delta_long, zoom):
    smurl = r"http://a.tile.openstreetmap.org/{0}/{1}/{2}.png"  #url to get images from openstreetmap
    headers = {"User-Agent":"Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_4) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/83.0.4103.97 Safari/537.36"} #header to communicate with osm
    xmin, ymax = deg2num(lat_deg, lon_deg, zoom)
    xmax, ymin = deg2num(lat_deg + delta_lat, lon_deg + delta_long, zoom)

    bbox_ul = num2deg(xmin, ymin, zoom)
    bbox_ll = num2deg(xmin, ymax + 1, zoom)

    bbox_ur = num2deg(xmax + 1, ymin, zoom)
    bbox_lr = num2deg(xmax + 1, ymax +1, zoom)

    Cluster = Image.new('RGB',((xmax-xmin+1)*256-1,(ymax-ymin+1)*256-1))
    for xtile in range(xmin, xmax+1):
        for ytile in range(ymin,  ymax+1):
            try:
                imgurl=smurl.format(zoom, xtile, ytile) 
                print("Opening: " + imgurl)
                imgstr = requests.get(imgurl,headers=headers) #get specific url for image
                img = Image.open(io.BytesIO(imgstr.content))
                Cluster.paste(img, box=((xtile-xmin)*255 ,  (ytile-ymin)*255))  #insert image into cluster
            except Exception as e: 
                print(e)
                tile = None

    return Cluster, [bbox_ll[1], bbox_ll[0], bbox_ur[1], bbox_ur[0]]

##@brief Function to send all the desidered destinations to the planning service
# When the button Run is pressed this function creates json containing all the destination and send it to the planning service, that will respond with another json containing all the actions to be executed.
# The list of actions is then sent to robots_control.
# Finally the function wait for a reply from robots_control to calculate the return path.
def button_callback(event):
    try:
        if(len(target_list)==0):
            print("Please insert at least one point to the map!")
        else:
            #time.sleep(5)
            temp = 0
            myjson={    #create json to send
                "coordinates":{
                    "lat":origin_position_lat,
                    "lon":origin_position_lon
                },
                "sizes":{
                    "height": 3500,
                    "width": 3500
                },
                "sensibility": 350,
                "pos_agv":{
                    "x": distance_origin_spawn_x,
                    "y": distance_origin_spawn_y
                },
                "targets":[],
            }

            for (x,y) in target_list:
                gazebo = xyDistance(origin_position_lat,y,origin_position_lon,x) #calculate distanze in meters from the spawn point 
                myjson["targets"].append({
                    "x":gazebo[0],
                    "y":gazebo[1],
                    "z":5.0
                })
            myjson["gui"]="false"

            print("Sending destination(s) to planning service, it may take a while to get a response back.")
            
            #send to planning service the destination points
            response = minimal_client.send_request(json.dumps(myjson))  

            #get all intermediate point from planning service
            temp = json.loads(response.out_json)["actions"]

            
            for i in range(len(temp)):
                temp[i]["x"] , temp[i]["y"] = temp[i]["y"] , temp[i]["x"]
                temp[i]["x"] -= distance_origin_spawn_y
                temp[i]["y"] -= distance_origin_spawn_x
                temp[i]["z"] -= spawn_height

            #send points to robots_control
            minimal_publisher.send_data(temp)   

            #start linstening for return requests
            minimal_subscriber = MinimalSubscriber()
            rclpy.spin(minimal_subscriber)
    except Exception as e:
        print(e)

## Main function called on startup
def main(args=None):
    rclpy.init(args=args)

    global minimal_client 
    minimal_client = MinimalClientAsync()

    global minimal_publisher
    minimal_publisher = MinimalPublisher()

    #variables to pass to getImageCluster    
    lat_deg, lon_deg, delta_lat,  delta_long, zoom = origin_position_lat-0.005, origin_position_lon,((research_area_x/1000.0)/111.121),((research_area_y/1000.0)/111.121), 16 #0.03, 0.03, 16
    
    a, bbox = getImageCluster(lat_deg, lon_deg, delta_lat,  delta_long, zoom)   

    fig = plt.figure(figsize=(10, 10))

    brun = Button(plt.axes([0.5, 0.05, 0.05, 0.035]),"Run")
    brun.on_clicked(button_callback)

    fig.canvas.mpl_connect("button_press_event",onclick)    #add event handler for user click
    ax = plt.subplot(111)

    global m
    m = Basemap(    #create map
        llcrnrlon=bbox[0], llcrnrlat=bbox[1],
        urcrnrlon=bbox[2], urcrnrlat=bbox[3],
        projection='merc', ax=ax, lat_0=lat_deg ,lon_0=lon_deg   #projection='merc', ax=ax, lat_0=45.76 ,lon_0=4.22
    ) 

    m.drawparallels(np.arange(origin_position_lat-0.5,origin_position_lat+(research_area_x/1000.0)/111.121+0.5,0.01),labels=[1,1,0,1])
    m.drawmeridians(np.arange(origin_position_lon-0.5,origin_position_lon+(research_area_y/1000.0)/111.121,0.01),labels=[1,1,0,1])


    m.imshow(a, interpolation='lanczos', origin='upper')    

    x_spawn,y_spawn =m(spawn_lon,spawn_lat)
    plt.scatter(x_spawn,y_spawn,alpha=0.9)  ##crate a pinpoint on the map
    plt.annotate("Starting location",(x_spawn,y_spawn))

    global distance_origin_spawn_x, distance_origin_spawn_y
    #distance_origin_spawn_x, distance_origin_spawn_y = xyDistance(origin_position_lat,spawn_lat,origin_position_lon,spawn_lon)
    distance_origin_spawn_x = research_area_x/2.0 -700.0
    distance_origin_spawn_y = research_area_y/2.0 + 700.0 

    global x_start,y_start  #calculate origin position from longitude and latitude
    x_start,y_start =m(origin_position_lon,origin_position_lat)
    rect = patches.Rectangle((x_start,y_start), research_area_x, research_area_y, linewidth=1, edgecolor='r', facecolor='none') #draw a rectangle to delimit reaearch area
    ax.add_patch(rect)

    plt.show()