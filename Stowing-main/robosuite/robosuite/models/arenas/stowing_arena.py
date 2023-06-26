import numpy as np

from robosuite.models.arenas import Arena
from robosuite.utils.mjcf_utils import array_to_string, xml_path_completion
from robosuite.models.arenas import TableArena


class StowingArena(TableArena):
    """
    Workspace that contains two bins placed side by side.

    Args:
        shelf_pos (3-tuple): (x,y,z) position to place shelf
        table_full_size (3-tuple): (L,W,H) full dimensions of the table
        table_friction (3-tuple): (sliding, torsional, rolling) friction parameters of the table
    """

    def __init__(
        self, shelf_pos, table_full_size, table_friction, table_offset=(0, 0, 0),
    ):
        super().__init__(
            table_full_size=table_full_size,
            table_friction=table_friction,
            table_offset=table_offset,
            xml="arenas/stowing_arena.xml",
        )


        self.table_friction = table_friction

        self.shelf_body = self.worldbody.find("./body[@name='shelf']")
        # self.table_top_abs = np.array(shelf_pos)

        self.configure_location()

    def configure_location(self):
        """Configures correct locations for this arena"""
        self.floor.set("pos", array_to_string(self.bottom_pos))
