import os
# import configparser
import math
# import fiona
from shapely.ops import transform
from shapely.geometry import Point, mapping, shape, Polygon, MultiPolygon
# from functools import partial
from rtree import index
# import pyproj
import numpy as np
import matplotlib.pyplot as plt
from descartes import PolygonPatch

# from collections import OrderedDict


def calculate_polygons(startx, starty, endx, endy, radius):
    """

    Calculate a grid of hexagon coordinates of the given radius
    given lower-left and upper-right coordinates. Returns a
    list of lists containing 6 tuples of x, y point coordinates.
    These can be used to construct valid regular hexagonal polygons
    Projected coordinates are advised.

    Parameters
    ----------
    startx : float
        Starting coordinate x.
    starty : float
        Starting coordinate y.
    endx : float
        Ending coordinate x.
    endy : float
        Ending coordinate y.
    radius : int
        Given radius of site areas.

    Returns
    -------
    polygons : list of lists
        A list containing multiple polygons. Each individual polygon
        is a list of tuple coordinates.

    """
    # calculate side length given radius
    sl = (2 * radius) * math.tan(math.pi / 6)

    # calculate radius for a given side-length
    # (a * (math.cos(math.pi / 6) / math.sin(math.pi / 6)) / 2)
    # see http://www.calculatorsoup.com/calculators/geometry-plane/polygon.php

    # calculate coordinates of the hexagon points
    # sin(30)
    p = sl * 0.5
    b = sl * math.cos(math.radians(30))
    w = b * 2
    h = 2 * sl

    # offset start and end coordinates by hex widths and heights to guarantee
    # coverage
    startx = startx - w
    starty = starty - h
    endx = endx + w
    endy = endy + h

    origx = startx
    origy = starty

    # offsets for moving along and up rows
    xoffset = b
    yoffset = 3 * p

    polygons = []
    row = 1
    counter = 0

    while starty < endy:

        if row % 2 == 0:
            startx = origx + xoffset

        else:
            startx = origx

        while startx < endx:
            p1x = startx
            p1y = starty + p
            p2x = startx
            p2y = starty + (3 * p)
            p3x = startx + b
            p3y = starty + h
            p4x = startx + w
            p4y = starty + (3 * p)
            p5x = startx + w
            p5y = starty + p
            p6x = startx + b
            p6y = starty
            poly = [
                (p1x, p1y),
                (p2x, p2y),
                (p3x, p3y),
                (p4x, p4y),
                (p5x, p5y),
                (p6x, p6y),
                (p1x, p1y)]

            polygons.append(poly)

            counter += 1
            startx += w

        starty += yoffset
        row += 1

    return polygons


def find_closest_site_areas(hexagons, geom_shape):
    """

    Get the transmitter and interfering site areas, by finding the closest
    hex shapes. The first closest hex shape to the transmitter will be the
    transmitter's site area. The next closest hex areas will be the
    intefering site areas.

    Parameters
    ----------
    hexagons : list of dicts
        Each haxagon is a geojson dict.
    geom_shape : Shapely geometry object
        Geometry object for the transmitter.

    Returns
    -------
    site_area : List of dicts
        Contains the geojson site area for the transmitter.
    interfering_site_areas : List of dicts
        Contains the geojson interfering site areas.

    """
    idx = index.Index()

    for site in hexagons:
        coords = site['centroid']
        idx.insert(0, coords.bounds, site)

    transmitter = mapping(geom_shape.centroid)

    site_area =  list(
        idx.nearest(
            (transmitter['coordinates'][0],
            transmitter['coordinates'][1],
            transmitter['coordinates'][0],
            transmitter['coordinates'][1]),
            1, objects='raw')
            )[0]

    closest_site_area_centroid = Polygon(
        site_area['geometry']['coordinates'][0]
        ).centroid

    all_closest_sites =  list(
        idx.nearest(
            closest_site_area_centroid.bounds,
            7, objects='raw')
            )

    interfering_site_areas = all_closest_sites[1:7]

    site_area = []
    site_area.append(all_closest_sites[0])

    return site_area, interfering_site_areas

def draw_single_polygon(axe, polygon, coords, color="blue"):
    # get min and max coords
    coord = polygon.exterior.coords.xy
    coords[0] += coord[0]
    coords[1] += coord[1]
    # add patch
    ploygon_patch = PolygonPatch(polygon,fc=color, ec=color, alpha=0.2) # alpha is transparent ratio
    axe.add_patch(ploygon_patch)


def plot_multi_polygon(polygons, title=" ", color="blue"):

    fig = plt.figure()
    axe = fig.add_subplot(1,1,1)
    coords = [[], []]
    for poly in polygons:
        draw_single_polygon(axe, poly, coords, color)

    axe.set_xlim(np.min(coords[0]), np.max(coords[0]))
    axe.set_ylim(np.min(coords[1]), np.max(coords[1]))

    axe.title.set_text(title)
    plt.show()

def plot_polygon(polygon, title=" ", color="blue"):

    fig = plt.figure()
    axe = fig.add_subplot(1,1,1)
    coords = [[], []]
    if isinstance(polygon, MultiPolygon):
        for poly in polygon:
            draw_single_polygon(axe, poly, coords, color)
    elif isinstance(polygon, Polygon):
        draw_single_polygon(axe, polygon, coords, color)
    else:
        raise ValueError("the input is not polygon!")

    axe.set_xlim(np.min(coords[0]), np.max(coords[0]))
    axe.set_ylim(np.min(coords[1]), np.max(coords[1]))

    axe.title.set_text(title)
    plt.show()

def generate_single_site_and_areas(coord, site_radius):
    """

    Meta function to produce a set of hex shapes with a specific site_radius.

    Parameters
    ----------
    unprojected_point : Tuple
        x and y coordinates for an unprojected point.
    site_radius : int
        Distance between transmitter and site edge in meters.

    Returns
    -------
    transmitter : List of dicts
        Contains a geojson dict for the transmitter site.
    interfering_transmitters : List of dicts
        Contains multiple geojson dicts for the interfering transmitter sites.
    site_area : List of dicts
        Contains a geojson dict for the transmitter site area.
    interfering_site_areas : List of dicts
        Contains multiple geojson dicts for the interfering transmitter site
        areas.

    """

    point = {
        'type': 'Feature',
        'geometry': {
            "type": "Point",
            "coordinates": [0,0],
        },
        'properties': 'Crystal Palace Radio Tower'
        }

    geom_shape = shape(point['geometry'])

    buffered = Polygon(geom_shape.buffer(site_radius*2).exterior)

    polygon = calculate_polygons(
        buffered.bounds[0], buffered.bounds[1],
        buffered.bounds[2], buffered.bounds[3],
        site_radius)

    hexagons = []
    id_num = 0
    for poly in polygon:
        hexagons.append({
                        'type': 'Feature',
                        'geometry': {
                            'type': 'Polygon',
                            'coordinates': [poly],
                        },
                        'centroid': (Polygon(poly).centroid),
                        'properties': {
                            'site_id': id_num
                            }
                        })
        id_num += 1

    site_area, interfering_site_areas = find_closest_site_areas(
        hexagons, geom_shape
    )
    # draw back the centroid of site and the interfering sites

    coord_offset = np.array([site_area[0]['centroid'].coords.xy[0][0], site_area[0]['centroid'].coords.xy[1][0]])
    # the main site
    site_area[0]["geometry"]["coordinates"][0] += - coord_offset + np.array(coord)
    # the interfering sites
    for isite_area in interfering_site_areas:
        isite_area["geometry"]["coordinates"][0] += - coord_offset + np.array(coord)
     
    # # following codes for plotting polygon
    # Interfer_Poly = []
    # for site in interfering_site_areas:
    #     Interfer_Poly.append(Polygon(site["geometry"]["coordinates"][0]))
    # # plot_polygon(Polygon(site_area[0]["geometry"]["coordinates"][0]))
    # plot_multi_polygon(Interfer_Poly)

    ## generate main transmitter
    site_area_site = Polygon(
        site_area[0]['geometry']['coordinates'][0]
        ).centroid

    transmitter = {
        'type': 'Feature',
        'geometry': mapping(site_area_site),
        'properties': {
            'site_id': 'transmitter'
        }
    }
    # generate interfering transmitters
    interfering_transmitters = []
    for interfering_site in interfering_site_areas:
        poly_centriod = Polygon(interfering_site['geometry']['coordinates'][0]).centroid
        interfering_transmitters.append({
            'type': 'Feature',
            'geometry': mapping(poly_centriod),
            'properties': {
                'site_id': interfering_site['properties']['site_id']
            }
        })

    # following codes for plotting polygon
    # cents = []
    # for transmiter in interfering_transmitters:
    #     cents.append(transmiter["geometry"]["coordinates"])
    # cents = np.array(cents)
    # plt.scatter(cents[:,0], cents[:,1])
    # plt.show()

    return transmitter, interfering_transmitters, site_area, interfering_site_areas