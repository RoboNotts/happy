#!/usr/bin/env python3

class WorldConfig(object):

    def __init__(self):

        super(WorldConfig, self).__init__()

        self.world_objects = []
        self._populate_objects()
        
    def _populate_objects(self):

      ground_plane_offset = -0.63 # from world in z coord

      # TEMPORARY HACK TO PREVENT COLLISION WITH ROBOT

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

      #   layer1_d, layer1_w, layer1_h = 0.43, 0.15, 0.12
      #   layer2_d, layer2_w, layer2_h = 0.30, 0.15, 0.10
      #   layer3_d, layer3_w, layer3_h = 0.20, 0.15, 0.10
      #   layer4_d, layer4_w, layer4_h = 0.10, 0.15, 0.10


      #   self.world_objects.append({
      #      'target': False,
      #      'name':"mounting_bracket", 
      #      'x': 0, 'y':0., 'z': -0.17,
      #      'dims': (0.44, 0.12, 0.35)
      #   })

      #   self.world_objects.append({
      #      'target': False,
      #      'name':"leg1_layer2", 
      #      'x':robot_base_d / 2 + layer2_d / 2, 'y':0.05, 'z': ground_plane_offset + layer1_h + layer2_h/2,
      #      'dims': (layer2_d, layer2_w, layer2_h)
      #   })

      #   self.world_objects.append({
      #      'target': False,
      #      'name':"leg1_layer3", 
      #      'x':robot_base_d / 2 + layer3_d / 2, 'y':0.05, 'z': ground_plane_offset + layer1_h + layer2_h + layer3_h/2,
      #      'dims': (layer3_d, layer3_w, layer3_h)
      #   })

      #   self.world_objects.append({
      #      'target': False,
      #      'name':"leg1_layer4", 
      #      'x':robot_base_d / 2 + layer4_d / 2, 'y':0.05, 'z': ground_plane_offset + layer1_h + layer2_h + layer3_h + layer4_h/2,
      #      'dims': (layer4_d, layer4_w, layer4_h)
      #   })


      #   self.world_objects.append({
      #      'target': False,
      #      'name':"leg2_layer1", 
      #      'x':-(robot_base_d / 2 + layer1_d / 2), 'y':0.05, 'z': ground_plane_offset + layer1_h/2,
      #      'dims': (layer1_d, layer1_w, layer1_h)
      #   })

      #   self.world_objects.append({
      #      'target': False,
      #      'name':"leg2_layer2", 
      #      'x':-(robot_base_d / 2 + layer2_d / 2), 'y':0.05, 'z': ground_plane_offset + layer1_h + layer2_h/2,
      #      'dims': (layer2_d, layer2_w, layer2_h)
      #   })

      #   self.world_objects.append({
      #      'target': False,
      #      'name':"leg2_layer3", 
      #      'x':-(robot_base_d / 2 + layer3_d / 2), 'y':0.05, 'z': ground_plane_offset + layer1_h + layer2_h + layer3_h/2,
      #      'dims': (layer3_d, layer3_w, layer3_h)
      #   })

      #   self.world_objects.append({
      #      'target': False,
      #      'name':"leg2_layer4", 
      #      'x':-(robot_base_d / 2 + layer4_d / 2), 'y':0.05, 'z': ground_plane_offset + layer1_h + layer2_h + layer3_h + layer4_h/2,
      #      'dims': (layer4_d, layer4_w, layer4_h)
      #   })

      #   layer1_d, layer1_w, layer1_h = 0.08, 0.43, 0.12
      #   layer2_d, layer2_w, layer2_h = 0.08, 0.30, 0.10
      #   layer3_d, layer3_w, layer3_h = 0.08, 0.20, 0.10
      #   layer4_d, layer4_w, layer4_h = 0.08, 0.10, 0.10

      #   self.world_objects.append({
      #      'target': False,
      #      'name':"leg3_layer1", 
      #      'x':0.03, 'y':robot_base_d / 2 + layer1_w/2, 'z': ground_plane_offset + layer1_h/2,
      #      'dims': (layer1_d, layer1_w, layer1_h)
      #   })

      #   self.world_objects.append({
      #      'target': False,
      #      'name':"leg3_layer2", 
      #      'x':0.03, 'y':robot_base_d / 2 + layer2_w/2, 'z': ground_plane_offset + layer1_h + layer2_h/2,
      #      'dims': (layer2_d, layer2_w, layer2_h)
      #   })

      #   self.world_objects.append({
      #      'target': False,
      #      'name':"leg3_layer3", 
      #      'x':0.03, 'y':robot_base_d / 2 + layer3_w/2, 'z': ground_plane_offset + layer1_h + layer2_h + layer3_h/2,
      #      'dims': (layer3_d, layer3_w, layer3_h)
      #   })

      #   self.world_objects.append({
      #      'target': False,
      #      'name':"leg3_layer4", 
      #      'x':0.03, 'y':robot_base_d / 2 + layer4_w/2, 'z': ground_plane_offset + layer1_h + layer2_h + layer3_h + layer4_h/2,
      #      'dims': (layer4_d, layer4_w, layer4_h)
      #   })


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
