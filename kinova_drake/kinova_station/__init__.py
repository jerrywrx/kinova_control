from kinova_station.simulation_station import KinovaStation
from kinova_station.common import EndEffectorTarget, GripperTarget
try:
    from kinova_station.hardware_station import KinovaStationHardwareInterface
except ImportError:
    # Don't import the hardware interface if the kortex API isn't installed
    print("Kortex API not decected: disabling hardware interface")
    pass
