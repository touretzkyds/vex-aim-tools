from .worldmap import WallSpec

# Disabled ArUco ids: 17 and 37.  Don't use them.

default_wall_marker_dict = dict()

def make_walls():

    # side +1 measures x from the left edge rightward
    # side -1 measures x from the right edge leftward

    w1 = WallSpec(default_wall_marker_dict, length=300, height=190,
                  marker_specs = { 7 : {'side': +1, 'x':  25, 'y': 35},
                                   2 : {'side': +1, 'x':  75, 'y': 35},
                                   3 : {'side': +1, 'x': 222, 'y': 35},
                                   8 : {'side': +1, 'x': 270, 'y': 35},

                                   5 : {'side': -1, 'x':  43, 'y': 20},
                                   4 : {'side': -1, 'x':  93, 'y': 20},
                                   6 : {'side': -1, 'x': 208, 'y': 20},
                                   9 : {'side': -1, 'x': 260, 'y': 35}, },
                  doorways = { 0 : {'x': 150, 'width': 85, 'height': 115} }
                  )


make_walls()
