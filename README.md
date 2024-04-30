# OSM to Lanelet Conversion

## For Converting OSM to lanelet, there are two steps involved
First we will convert OSM map to CommonRoad
 Then we will convert CommonRoad to Lanelet Map

<br />

## Steps involved 
1. activate the conda environment
  > - Type the command :- `conda activate lanelet_to_OSM`
2. use the command :-  `crdesigner map-convert-osm -i input-file.osm -o output-file.xml`
  > - this command will convert OSM map to Commonroad. If you want to see the CommonRoad map you can use the CommonRoad Scenario Designer GUI
  > - first run :- `LD_LIBRARY_PATH=/home/naru/miniconda3/lib:$LD_LIBRARY_PATH`
  > - make sure you are in the directory :-`/home/naru/calibration_18feb2022/lanelet2/commonroad-scenario-designer`
  > - now type :- `crdesigner`. You can use this GUI to see your map in commonroad format
3. now use the command  `crdesigner map-convert-lanelet -i output-file.xml -o output-file-lanelet.osm -c`
  > - this command will convert the Commonroad map to Lanelet



# OSM Example
![](https://i.imgur.com/npiKEOI.png)
# Lanelet map example
![](https://i.imgur.com/kWX0AZl.png)




<br />

## For OSM to CommonRoad conversion

The conversion process consists currently out of three stages:
1. ### osm to road_graph
  > - In the first stage all information from the .osm file is extracted and a road graph is created
  > - This procedure is described in detail by the original thesis written by Maximilian.
  > - Later on, the creation of traffic signs and traffic lights were also added to this stage, since they could be parsed from the .osm file.
  > - All files which are used during this converting stage can be found in /osm_operations and /graph_operations.
2. ### road_graph to intermediate_format
  > - The intermediate format was added to perform operations on the road graph easier.
  > -  In this stage intersections with lane specific data are created (trough lane, turn right, turn left, . . .).
  > -  Also, intersections are enhanced and traffic lights are added, which were missing in the initial .osm file.
  > - All related files can be found in /intermediate_operations.
3. ### intermediate_format to cr_scenario
  > - In the last stage the intermediate format is exported to a commonroad scenario.
  > - During this process checks for converting errors are performed.
  > - Also, the benchmark ID and other scenario tags are added.
  > - All files for this stage can be found in /cr_operations.

![](https://i.imgur.com/FZq8GMb.png)



<br />

## Important Files and Directories
> - `/osm_operations` : All files regarding information extraction from the given .osm file.
> - `/graph_operations` : Files that are needed to create a road_graph object.
> - `/intermediate_operations` : Files used for creating the intermediate format.
> - `/cr_operations` : Files for exporting and creating the the final a commonroad scenario.
> - `config.py` : The config file contains all settings related to the conversion process.
> - `converter.py` : This file orchestrates the whole conversion. It calls the different stages described earlier during the conversion process.
> - `/utility` : This directory contains various tools and files that are used throughout all stages, such as the ID generator for all elements in the final commonroad scenario.
> - `/visulization` : Files that can be used for visualization of the final commonroad scenario can be found here.

## Code correction

1. There was issue with parsing the osm file, so in directory : `/home/naru/miniconda3/envs/lanelet_to_OSM/lib/python3.8/site-packages/commonroad_scenario_designer-0.5.1-py3.8.egg/crdesigner/map_conversion/osm2cr/converter_modules/osm_operations`,Any of these two changes can be done in `osm parser.py` in order for the parser file to function properly.
>>  - in function `get_graph_edges_from_road()` we can directly append points in waypoint list.

>>```py
>>for index, point in enumerate(point_list):
>>    ##loading only inside of bounds
>>    # #if point_in_area_list[index]:
>>    #     # point is added
>>    #     #waypoints.append(point)
>>    # #elif neighbor_in_area(index, point_in_area_list):
>>    #     # point is added, but edge is split
>>    #     #outside_waypoints.add(point.id)
>>    #     #waypoints.append(point)
>>    #     #nodes[point.id] = rg.GraphNode(point.id, point.x, point.y, OrderedSet())
>>    # #else:
>>    #     # point is not added
>>    #     #pass
>>    waypoints.append(point)
>>```        
>> - OR a possible fix is to change the code of the function get_area_from_bounds() in osm_parser.py to the following:

>> ```py
>> max_point = lon_lat_to_cartesian(np.array([bounds[0], bounds[3]]), np.array(origin)[::-1])
>> min_point = lon_lat_to_cartesian(np.array([bounds[2], bounds[1]]), np.array(origin)[::-1])
>> ```

>> - either of these changes will fix the issue of parsing

<br />

2. While conversion, the latitude, longitude of OSM was not same as that of Lanelet so inorder to fix this issue we are first saving Latitude_centre and Longitude_centre in osm_parser.py function get_points() as IIIT.npy

```py

def get_points(nodes: Dict[int, ElTree.Element], custom_bounds=None) \
        -> Tuple[Dict[int, Point], Tuple[float, float], Bounds]:
    """
    projects a set of osm nodes on a plane and returns their positions on that plane as Points

    :param custom_bounds:
    :param nodes: dict of osm nodes
    :type nodes: Dict[int, ElTree.Element]
    :return: dict of points
    :rtype: Dict[int, Point]
    """
    if len(nodes) < 1:
        raise ValueError("Map is empty")
    ids = []
    lons = []
    lats = []
    for node_id, node in nodes.items():
        ids.append(node_id)
        lons.append(float(node.attrib["lon"]))
        lats.append(float(node.attrib["lat"]))
    if custom_bounds is not None:
        bounds = custom_bounds
    else:
        bounds = max(lats), min(lons), min(lats), max(lons)
    assert bounds[0] >= bounds[2]
    assert bounds[3] >= bounds[1]
    lon_center = (bounds[1] + bounds[3]) / 2
    lat_center = (bounds[0] + bounds[2]) / 2
    lons = np.array(lons)
    lats = np.array(lats)
    lons_d = lons - lon_center
    lats_d = lats - lat_center
    points = OrderedDict()
    lon_constants = np.pi / 180 * config.EARTH_RADIUS * np.cos(np.radians(lats))
    x = lon_constants * lons_d
    lat_constant = np.pi / 180 * config.EARTH_RADIUS
    y = lat_constant * lats_d

    for index, point_id in enumerate(ids):
        points[int(point_id)] = Point(int(point_id), x[index], y[index])
    logging.info("{} required nodes found".format(len(points)))
    center_point = lat_center, lon_center
    np.save("IIIT.npy",[lat_center,lon_center])

    return points, center_point, bounds
```
now in /home/naru/miniconda3/envs/lanelet_to_OSM/lib/python3.8/site-packages/commonroad_scenario_designer-0.5.1-py3.8.egg/crdesigner/map_conversion/lanelet_lanelet2
/`cr2lanelet.py`  we need to create a function named 
`cartesian_inverse()` 

```py
def cartesian_inverse(x_coord,y_coord):
    center_array = np.load("IIIT.npy")
    lat_center,lon_center = center_array[0],center_array[1]
    lat_constant = np.pi / 180 * config.EARTH_RADIUS
    lat_d = y_coord/lat_constant
    lats = lat_d + lat_center 

    lon_constant = np.pi / 180 * config.EARTH_RADIUS * np.cos(np.radians(lats))

    lon_d = x_coord/lon_constant

    return lats, lon_d + lon_center
```

and we will call this function from `_create_nodes_from_vertices()` in `cr2lanelet.py` 
```py
    def _create_nodes_from_vertices(self, vertices: List[np.ndarray]) -> List[str]:
        """Create nodes and add them to the OSM.

        Args:
          vertices: List of vertices from a lanelet boundary.
        Returns:
          Ids of nodes which were created.
        """
        nodes = []
        for vertice in vertices:
            #lon, lat = self.proj(vertice[0], vertice[1], inverse=True)
            lat, lon = cartesian_inverse(vertice[0],vertice[1])
            node = Node(self.id_count, lat, lon)
            nodes.append(node.id_)
            self.osm.add_node(node)
        return nodes
```

This will fix the Latitude and Longitude issue
<br />

"# Lanelet_conversion" 
