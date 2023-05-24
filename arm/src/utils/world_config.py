#!/usr/bin/env python3

class WorldConfig(object):

    def __init__(self):

        super(WorldConfig, self).__init__()

        self.world_objects = []
        self._populate_objects()
        
    def _populate_objects(self):

      ground_plane_offset = -0.63 # from world in z coord

      base_d, base_w, base_h = 0.56, 0.56, 0.63
      self.world_objects.append({
         'target': False,
         'name':"base", 
         'x':0, 'y':-.10, 'z': ground_plane_offset + base_h/2,
         'dims': (base_d, base_w, base_h)
      }) 

      camera_pole_d, camera_pole_w, camera_pole_h = 0.19, 0.12, 0.60
      self.world_objects.append({
         'target': False,
         'name':"camera", 
         'x':-.185, 'y':-.245, 'z': .30,
         'dims': (camera_pole_d, camera_pole_w, camera_pole_h)
      }) 

    def get_objects(self):
        return self.world_objects

    def get_target_coords(self):
        for obj in self.world_objects:
            if obj['target'] is True:
                return obj['x'], obj['y'], obj['z'] 

        return False

    def get_target_dims(self):
        for obj in self.world_objects:
            if obj['target'] is True:
                return obj['dims'] 

        return False
