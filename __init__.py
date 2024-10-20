import sys
import os

from .simbody import *
from .common import *
from .simulation import *
from .actuators import *
from .analyses import *
from .tools import *
from .examplecomponents import *
from .moco import *
from . import report
from .version import __version__

cwd = os.path.dirname(__file__)
geometry_path = os.path.join(cwd, 'Geometry')
ModelVisualizer.addDirToGeometrySearchPaths(geometry_path)
