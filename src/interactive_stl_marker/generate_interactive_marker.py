from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from math import sqrt

class GenerateInteractiveMarkers:

    def __init__(self, server):
        self._server = server
        self._positions = list()

        self._stl_file_path = rospy.get_param("stl_file_path")
        self._frame_id = rospy.get_param("frame_id")
        self._init_x_position = rospy.get_param("init_x_position")
        self._init_y_position = rospy.get_param("init_y_position")
        self._init_z_position = rospy.get_param("init_z_position")

        self._x_scale = rospy.get_param("x_scale")
        self._y_scale = rospy.get_param("y_scale")
        self._z_scale = rospy.get_param("z_scale")

    def processFeedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            #compute difference vector for this cube
            x = feedback.pose.position.x
            y = feedback.pose.position.y
            z = feedback.pose.position.z
            ow = feedback.pose.orientation.w
            ox = feedback.pose.orientation.x
            oy = feedback.pose.orientation.y
            oz = feedback.pose.orientation.z

            index = int(feedback.marker_name)

            if index > len(self._positions):
                return

            dx = x - self._positions[index][0]
            dy = y - self._positions[index][1]
            dz = z - self._positions[index][2]

            # move all markers in that direction
            for i in range(len(self._positions)):
                (mx, my, mz) = self._positions[i]
                d = sqrt(sqrt((x - mx)**2 + (y - my)**2)**2 + (z-mz)**2)
                t = 1 / (d*5.0+1.0) - 0.2
                if t < 0.0: 
                    t=0.0
                self._positions[i][0] += t*dx
                self._positions[i][1] += t*dy
                self._positions[i][2] += t*dz

                if i == index:
                #   rospy.loginfo( d )
                  self._positions[i][0] = x
                  self._positions[i][1] = y
                  self._positions[i][2] = z

                pose = geometry_msgs.msg.Pose()
                pose.position.x = self._positions[i][0]
                pose.position.y = self._positions[i][1]
                pose.position.z = self._positions[i][2]
                pose.orientation.w = ow
                pose.orientation.x = ox
                pose.orientation.y = oy
                pose.orientation.z = oz

                self._server.setPose( str(i), pose )
            self._server.applyChanges()

    def makeBoxControl(self, msg ):
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.orientation_mode = InteractiveMarkerControl.VIEW_FACING
        control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
        control.independent_marker_orientation = True

        marker = Marker()

    	# pet bottle
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = "package://interactive_stl_marker/" + self._stl_file_path
        
        marker.scale.x = self._x_scale
        marker.scale.y = self._y_scale
        marker.scale.z = self._z_scale

        marker.color.r = 0.65+0.7*msg.pose.position.x
        marker.color.g = 0.65+0.7*msg.pose.position.y
        marker.color.b = 0.65+0.7*msg.pose.position.z
        marker.color.a = 1.0

        control.markers.append( marker )
        msg.controls.append( control )
        return control

    def makeCube(self):
  
        marker = InteractiveMarker()
        marker.header.frame_id = self._frame_id
        marker.scale = 0.1

        marker.pose.position.x = self._init_x_position
        marker.pose.position.y = self._init_y_position
        marker.pose.position.z = self._init_z_position

        self._positions.append( [0,0,0] )

        marker.name = str(0)
        self.makeBoxControl(marker)

        self._server.insert( marker, self.processFeedback )
