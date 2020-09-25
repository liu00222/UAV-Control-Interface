# import packages
# import built-in packages
from os.path import join, dirname, abspath
import socket

# import kivy package
from kivy.app import App
from kivy.graphics import *
from kivy.uix.widget import Widget
from kivy.uix.label import Label
from kivy.properties import *
from kivy.core.window import Window
from kivy.uix.floatlayout import FloatLayout
from kivy.clock import Clock
from kivy.uix.image import Image
from kivy.factory import Factory

# import external packages
from packages.kivy3.materials import Material
from packages.kivy3.scene import Scene
from packages.kivy3.geometries import BoxGeometry
from packages.kivy3.mesh import Mesh
from packages.kivy3.objloader import OBJLoader
from packages.kivy3.renderer import Renderer
from packages.kivy3.camera import PerspectiveCamera

# import local utilities
import src.util as util
from src.imglayout import Imglayout
from src.util import *
from src.robot import Robot
from src.glob import *
from src.mymath import Triangle2D, degree_to_radians
from src.tracer import Tracer3
import src.robot as robot

# path to the current folder
FOLDER = dirname(abspath(__file__))


def initialize_connect():
    global s
    s.connect_ex(('192.168.1.107', 3000))


class MainLayout(FloatLayout):
    message = StringProperty()
    words = StringProperty()

    def __init__(self, **kwargs):
        """
        Sample class

        Attributes:
            flight_speed     The maximum speed that such a bird can attain.
            nesting_grounds  The locale where these birds congregate to reproduce.
        """
        super(MainLayout, self).__init__(**kwargs)

        # get the running application
        app = App.get_running_app()

        self.mode = VIEW_MODE
        self.message = ""
        self.words = "View Mode"
        self.robots_selected = []
        self.mouse_points = []
        self.robots_ids = []
        self.robot_list = []

        self.view_angle = 0

        # Set the connection to the server
        initialize_connect()
        self.s = s

        # Send greeting message to the server and save the feedback
        self.s.send("c".encode('utf-8'))
        data, addr = self.s.recvfrom(1024)
        self.num_robot = int(data.decode('utf-8'))
        world_name, addr = self.s.recvfrom(1024)
        self.world_name = world_name.decode('utf-8')

        # Initialize tracer
        self.tracer = Tracer3(self.s)

        # Initialize the robots settings and the 3D environment
        self.initialize_robot()
        self.initialize_display(app)

    def add_robot(self, keyword):
        if keyword == 0:
            self.robot_list.append(Robot(str(0), 0.0, 1.0*coordinate_multiplier, 1.0*coordinate_multiplier, 0.1, 0.1, 0.1, UAV))
        elif keyword == 1:
            self.robot_list.append(Robot(str(1), 0.0, 1.0*coordinate_multiplier, 0.0, 0.1, 0.1, 0.1, UAV))
        elif keyword == 2:
            self.robot_list.append(Robot(str(2), 0.0, 1.0*coordinate_multiplier, -1.0*coordinate_multiplier, 0.1, 0.1, 0.1, UAV))
        elif keyword == 3:
            self.robot_list.append(Robot(str(3), 0.0, 1.0*coordinate_multiplier, 2.0*coordinate_multiplier, 0.1, 0.1, 0.1, UAV))
        elif keyword == 4:
            self.robot_list.append(Robot(str(4), 0.0, 1.0*coordinate_multiplier, -2.0*coordinate_multiplier, 0.1, 0.1, 0.1, UAV))
        elif keyword == -1:
            self.robot_list.append(Robot('t', 2.0*coordinate_multiplier, 0.0, -2.0*coordinate_multiplier, 0.1, 0.1, 0.1, UAV))
        return

    def initialize_display(self, app):
        # Deploy the box picture shown at the top-left cornor
        c = Imglayout(
            pos_hint={"center_x": 0, "center_y": 0},
            size_hint=(0.1, 0.1)
        )
        self.add_widget(c)
        app.im.keep_ratio = False
        app.im.allow_stretch = False
        c.add_widget(app.im)

        # Set the 3D environment
        with self.canvas.before:
            # Initialize the render and scene
            app.renderer = Renderer()
            scene = Scene()
            loader = OBJLoader()

            # Add the floor plane
            app.floor = Mesh(geometry=BoxGeometry(20, 0.1, 20),
                             material=Material(transparency=0.5,
                                               color=(0.663, 0.663, 0.663),
                                               specular=(.0, .0, .0)))
            app.floor.pos.y = -0.1
            scene.add(app.floor)

            # Add robots models
            app.cube = []

            global robot_num
            if "tracking" in self.world_name:
                robot_num = self.num_robot + 1

            for i in range(robot_num):
                self.robots_ids.append(self.robot_list[i].id)
                if i == robot_num - 1 and "tracking" in self.world_name:
                    if self.robot_list[i].type == UAV:
                        obj = loader.load(join(FOLDER, 'data', 'sphere.obj')).children
                        app.cube.extend(obj)

                        for j in range(i * OBJ_LENGTH, i * OBJ_LENGTH + TARGET_OBJ_LENGTH):
                            # Adjust the size of the robot
                            app.cube[j].scale = (0.07, 0.07, 0.07)

                            # Set the position of the robot
                            app.cube[j].pos.x = self.robot_list[i].x
                            app.cube[j].pos.y = self.robot_list[i].y
                            app.cube[j].pos.z = self.robot_list[i].z
                            scene.add(app.cube[j])
                else:
                    if self.robot_list[i].type == UAV:
                        obj = loader.load(join(FOLDER, 'data', 'round.obj')).children
                        app.cube.extend(obj)

                        for j in range(i * OBJ_LENGTH, (i + 1) * OBJ_LENGTH):
                            # Adjust the size of the robot

                            # round.obj
                            app.cube[j].scale = (0.005, 0.005, 0.005)

                            # Set the position of the robot
                            app.cube[j].pos.x = self.robot_list[i].x
                            app.cube[j].pos.y = self.robot_list[i].y
                            app.cube[j].pos.z = self.robot_list[i].z
                            app.cube[j].rotation.x = -90
                            scene.add(app.cube[j])

            if "tracking" in self.world_name:
                for j in range(TARGET_OBJ_LENGTH):
                    app.cube[(robot_num - 1) * OBJ_LENGTH + j].material.color = RED

            # self.add_obstacles(app, scene)

            # Set the perspective camera
            app.camera = PerspectiveCamera(fov=80, aspect=0, near=1, far=10000)
            app.camera.pos.x = 0
            app.camera.pos.y = 20
            app.camera.pos.z = 20
            app.camera.look_at(ORIGIN)

            # Start rendering the scene and camera
            app.renderer.render(scene, app.camera)
            app.renderer.bind(size=util.adjust_aspect)

            # Callback functions for animation
            if self.num_robot >= 1:
                Clock.schedule_interval(robot.update_robot_1_x, .01)
                Clock.schedule_interval(robot.update_robot_1_y, .01)
                Clock.schedule_interval(robot.update_robot_1_z, .01)
                Clock.schedule_interval(robot.receive_position_1, .01)

            if self.num_robot >= 2:
                Clock.schedule_interval(robot.update_robot_2_x, .01)
                Clock.schedule_interval(robot.update_robot_2_y, .01)
                Clock.schedule_interval(robot.update_robot_2_z, .01)

            if self.num_robot >= 3:
                Clock.schedule_interval(robot.update_robot_3_x, .01)
                Clock.schedule_interval(robot.update_robot_3_y, .01)
                Clock.schedule_interval(robot.update_robot_3_z, .01)

            if self.num_robot >= 4:
                Clock.schedule_interval(robot.update_robot_4_x, .01)
                Clock.schedule_interval(robot.update_robot_4_y, .01)
                Clock.schedule_interval(robot.update_robot_4_z, .01)

            if self.num_robot >= 5:
                Clock.schedule_interval(robot.update_robot_5_x, .01)
                Clock.schedule_interval(robot.update_robot_5_y, .01)
                Clock.schedule_interval(robot.update_robot_5_z, .01)

            if "tracking" in self.world_name:
                Clock.schedule_interval(robot.update_robot_t_x, .01)
                Clock.schedule_interval(robot.update_robot_t_y, .01)
                Clock.schedule_interval(robot.update_robot_t_z, .01)
        # Finally, add the the render to the current widget
        self.add_widget(app.renderer)

    def add_obstacles(self, app, scene):
        # Add obstacles
        app.b1 = Mesh(geometry=BoxGeometry(1, 8, 2),
                      material=Material(transparency=0.8,
                                        color=OBSTACLE_COLOR,
                                        specular=(.0, .0, .0)))
        app.b1.pos.y = 4-0.1
        app.b1.pos.x = 3.5 * coordinate_multiplier
        scene.add(app.b1)

        app.b2 = Mesh(geometry=BoxGeometry(1, 4, 2),
                      material=Material(transparency=0.8,
                                        color=OBSTACLE_COLOR,
                                        specular=(.0, .0, .0)))
        app.b2.pos.y = 2-0.1
        app.b2.pos.x = 1.5 * coordinate_multiplier
        app.b2.pos.z = -0.8 * coordinate_multiplier
        scene.add(app.b2)

        app.b3 = Mesh(geometry=BoxGeometry(1, 7, 2),
                      material=Material(transparency=0.8,
                                        color=OBSTACLE_COLOR,
                                        specular=(.0, .0, .0)))
        app.b3.pos.y = 3.5-0.1
        app.b3.pos.x = -0.9 * coordinate_multiplier
        app.b3.pos.z = -0.3 * coordinate_multiplier
        scene.add(app.b3)

        app.b4 = Mesh(geometry=BoxGeometry(1, 7, 2),
                      material=Material(transparency=0.8,
                                        color=OBSTACLE_COLOR,
                                        specular=(.0, .0, .0)))
        app.b4.pos.y = 3.5-0.1
        app.b4.pos.x = -3.2 * coordinate_multiplier
        app.b4.pos.z = 3 * coordinate_multiplier
        scene.add(app.b4)

        app.b5 = Mesh(geometry=BoxGeometry(1, 7, 2),
                      material=Material(transparency=0.8,
                                        color=OBSTACLE_COLOR,
                                        specular=(.0, .0, .0)))
        app.b5.pos.y = 3.5-0.1
        app.b5.pos.x = 3.2 * coordinate_multiplier
        app.b5.pos.z = 3 * coordinate_multiplier
        scene.add(app.b5)

    def initialize_robot(self):
        for i in range(self.num_robot):
            self.add_robot(i)
        if "tracking" in self.world_name:
            self.add_robot(-1)

    def start_tracking(self):
        self.s.send("tracking".encode('utf-8'))
        return

    def on_touch_down(self, touch):
        if self.mode != VIEW_MODE:
            with self.canvas.after:
                Color(1.0, 1.0, 0.1)
                touch.ud["line"] = Line(points=(touch.x, touch.y), width=5)
        if self.mode == GESTURE_MODE and len(self.mouse_points) > 5:
            self.tracer.touch_down_update(touch.x, touch.y)

        self.mouse_points = []

        return super(MainLayout, self).on_touch_down(touch)

    def on_touch_move(self, touch):
        if self.mode != VIEW_MODE:
            touch.ud["line"].points += (touch.x, touch.y)

        if self.mode == GESTURE_MODE and len(self.mouse_points) > 5:
            self.tracer.touch_move_update(touch.x, touch.y)

        if self.mode == VIEW_MODE:
            if len(self.mouse_points) >= 5:
                if touch.x > self.mouse_points[len(self.mouse_points) - 2][0]:
                    for _ in range(3):
                        self.view_angle += 1
                        self.adjust_view()
                elif touch.x < self.mouse_points[len(self.mouse_points) - 2][0]:
                    for _ in range(3):
                        self.view_angle -= 1
                        self.adjust_view()

                if touch.y > self.mouse_points[len(self.mouse_points) - 4][1] + 7:
                    app = App.get_running_app()
                    if app.camera.pos.y > 2:
                        app.camera.pos.y -= 1
                        app.camera.look_at(ORIGIN)
                elif touch.y < self.mouse_points[len(self.mouse_points) - 4][1] - 7:
                    app = App.get_running_app()
                    if app.camera.pos.y < 30:
                        app.camera.pos.y += 1
                        app.camera.look_at(ORIGIN)

        # record the touch points
        self.mouse_points.append([touch.x, touch.y])
        return super(MainLayout, self).on_touch_move(touch)

    def on_touch_up(self, touch):
        if self.mode == GESTURE_MODE and len(self.mouse_points) > 5:
            self.tracer.touch_up_update(touch.x, touch.y)

        if len(self.mouse_points) > 10:
            if self.mode == SELECTION_MODE:
                self.update_robots_selected()
                self.highlight_robots(self.robots_selected)

        # eliminate the drawing lines when finished
        if self.mode != VIEW_MODE:
            touch.ud["line"].points = []
        return super(MainLayout, self).on_touch_up(touch)

    def adjust_view(self):
        app = App.get_running_app()

        self.view_angle = int(self.view_angle)

        if self.view_angle >= 360:
            self.view_angle -= 360

        if self.view_angle < 0:
            self.view_angle = 360 + self.view_angle

        # Case 1
        if self.view_angle == 0:
            app.camera.pos.x = 0
            app.camera.pos.z = 20
            app.camera.look_at(ORIGIN)

        # Case 2
        if 0 < self.view_angle <= 90:
            x = (-20) * np.sin(degree_to_radians(self.view_angle))
            z = 20 * np.cos(degree_to_radians(self.view_angle))
            app.camera.pos = [x, app.camera.pos.y, z]
            app.camera.look_at(ORIGIN)

        # Case 3
        if 90 < self.view_angle < 180:
            x = (-20) * np.cos(degree_to_radians(self.view_angle - 90.))
            z = (-20) * np.sin(degree_to_radians(self.view_angle - 90.))
            app.camera.pos = [x, app.camera.pos.y, z]
            app.camera.look_at(ORIGIN)

        # Case 4
        if self.view_angle == 180:
            app.camera.pos = [0, app.camera.pos.y, -20]
            app.camera.look_at(ORIGIN)

        # Case 5
        if 180 < self.view_angle < 270:
            x = 20 * np.cos(degree_to_radians(270. - self.view_angle))
            z = (-20) * np.sin(degree_to_radians(270. - self.view_angle))
            app.camera.pos = [x, app.camera.pos.y, z]
            app.camera.look_at(ORIGIN)

        # Case 6
        if self.view_angle == 270:
            app.camera.pos.x = 20
            app.camera.pos.z = 0
            app.camera.pos = [20, app.camera.pos.y, 0]
            app.camera.look_at(ORIGIN)

        # Case 7
        if 270 < self.view_angle < 360:
            x = 20 * np.sin(degree_to_radians(360. - self.view_angle))
            z = 20 * np.cos(degree_to_radians(360. - self.view_angle))
            app.camera.pos = [x, app.camera.pos.y, z]
            app.camera.look_at(ORIGIN)

    def gesture_view_change(self):
        if self.mode == VIEW_MODE:
            self.mode = SELECTION_MODE
        elif self.mode == SELECTION_MODE:
            self.mode = VIEW_MODE

    def initialize_view(self):
        app = App.get_running_app()
        self.view_angle = 0
        app.camera.pos.x = 0
        app.camera.pos.y = 20
        app.camera.pos.z = 20
        app.camera.look_at(ORIGIN)

    def adjust_button_1(self):
        if self.mode == VIEW_MODE:
            self.mode = SELECTION_MODE
            self.words = 'Selection Mode'
            self.message = "Select UAVs: "

        elif self.mode == SELECTION_MODE:
            self.mode = GESTURE_MODE
            self.words = 'Command Mode'
            self.message = "Draw Gestures: "

        elif self.mode == GESTURE_MODE:
            self.mode = VIEW_MODE
            self.words = 'View Mode'
            self.message = ""
            self.de_highlight_robots(self.robots_ids)

    def update_robots_selected(self):
        app = App.get_running_app()
        model_view = util.kivy_matrix_to_np_matrix(app.camera.modelview_matrix, 4, 4)
        projection = util.kivy_matrix_to_np_matrix(app.camera.projection_matrix, 4, 4)
        points = []
        self.robots_selected = []

        for i in range(self.num_robot):
            points.append(self.to_window_coordinates(np.array([app.cube[i * OBJ_LENGTH].pos.x,
                                                               app.cube[i * OBJ_LENGTH].pos.y,
                                                               app.cube[i * OBJ_LENGTH].pos.z]),
                                                     model_view.transpose(),
                                                     projection.transpose()))

        for i in range(len(self.mouse_points) - 2):
            triangle = Triangle2D(self.mouse_points[0], self.mouse_points[i + 1], self.mouse_points[i + 2])
            for j in range(len(points)):
                if triangle.contains(points[j]) and str(j) not in self.robots_selected:
                    self.robots_selected.append(str(j))

    def to_window_coordinates(self, pos, model_view, projection):
        """
        Convert the 3d coordinates to 2d window coordinates

        :param pos: <np.array> 3d position of the robot
        :param model_view: <np.array> model_view matrix after transpose
        :param projection: <np.array> projection matrix matrix after transpose

        :return: 3d tuple that contains the converted coordinates
        """
        p = model_view.dot(np.array([pos[0], pos[1], pos[2], 1.0]))
        p = projection.dot(p)
        a, b, c, w = p[0], p[1], p[2], p[3]
        a, b, c = a / w, b / w, c / w
        return a * Window.size[0] / 2 + Window.size[0] / 2, b * Window.size[1] / 2 + Window.size[1] / 2, c

    def highlight_robots(self, robots_ids):
        """
        Highlight the robots selected

        :param robots_ids: <list> a list of ids of selected robots
        """
        app = App.get_running_app()
        select = ""
        cancel = ""
        for i in range(self.num_robot):
            if self.robot_list[i].id in robots_ids:
                x, y, z = app.camera.pos.x, app.camera.pos.y, app.camera.pos.z
                if app.cube[i * OBJ_LENGTH].material.color == WHITE:
                    for j in range(OBJ_LENGTH):
                        app.cube[i*OBJ_LENGTH + j].material.color = LIGHT_YELLOW
                    select += str(i) + " "
                else:
                    for j in range(OBJ_LENGTH):
                        app.cube[i*OBJ_LENGTH + j].material.color = WHITE
                    cancel += str(i) + " "
                util.stabilize_view(x, y, z)
        if select:
            self.s.send(("Sel" + str(select)).encode('utf-8'))
        if cancel:
            self.s.send(("Can" + str(cancel)).encode('utf-8'))

    def de_highlight_robots(self, robots_ids):
        """
        De-highlight the robots selected

        :param robots_ids: <list> a list of ids of selected robots
        """
        app = App.get_running_app()
        for i in range(len(self.robot_list)):
            if self.robot_list[i].id in robots_ids and self.robot_list[i].id != 't':
                for j in range(OBJ_LENGTH):
                    app.cube[i*OBJ_LENGTH + j].material.color = WHITE
        util.stabilize_view(app.camera.pos.x, app.camera.pos.y, app.camera.pos.z)


class DrawApp(App):
    # set the initial box picture
    im = Image(source='imgs/normal_box.png',
               x=Window.size[0] * (-17.0 / 960.0),
               y=Window.size[1] * (483.0 / 540.0),
               size=[1, 1])

    def build(self):
        return MainLayout()


if __name__ == "__main__":
    # set the window size and run the main program
    Window.size = (960, 540)
    DrawApp().run()
