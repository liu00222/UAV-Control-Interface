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
from src.mymath import Triangle2D
from src.tracer import Tracer3

# path to the current folder
FOLDER = dirname(abspath(__file__))


def initialize_connect():
    global s
    s.connect_ex(('192.168.1.107', 3000))


class MainLayout(FloatLayout):
    message = StringProperty()

    def initialize_display(self, app):
        # deploy the box picture shown at the top-left cornor
        c = Imglayout(
            pos_hint={"center_x": 0, "center_y": 0},
            size_hint=(0.1, 0.1)
        )
        self.add_widget(c)
        app.im.keep_ratio = False
        app.im.allow_stretch = False
        c.add_widget(app.im)

        # visualize the robots as cubes and initialize the camera
        with self.canvas.before:
            layout = FloatLayout()
            app.renderer = Renderer()
            scene = Scene()
            loader = OBJLoader()

            app.cube = [INITIAL_CUBE] * len(self.robot_list)

            app.floor = Mesh(geometry=BoxGeometry(10, 10, 0.1),
                             material=Material(transparency=0.5,
                                               color=(0.663, 0.663, 0.663),
                                               specular=(0.5, 0.5, 0.5)))
            scene.add(app.floor)

            for i in range(len(self.robot_list)):
                self.robots_ids.append(self.robot_list[i].id)
                if self.robot_list[i].type == UGV:
                    app.cube[i] = Mesh(geometry=BoxGeometry(self.robot_list[i].length,
                                                            self.robot_list[i].width,
                                                            self.robot_list[i].height
                                                            ),
                                       material=Material(transparency=0.8, color=WHITE, specular=(100, 0, 0))
                                       )
                elif self.robot_list[i].type == UAV:
                    obj = loader.load(join(FOLDER, 'data', 'sphere.obj'))
                    app.cube[i] = obj.children[0]
                    app.cube[i].scale = (0.1, 0.1, 0.1)

                # set the position of the object
                app.cube[i].pos.x = self.robot_list[i].x
                app.cube[i].pos.y = self.robot_list[i].y
                app.cube[i].pos.z = self.robot_list[i].z
                scene.add(app.cube[i])

            # create camera for scene
            app.camera = PerspectiveCamera(fov=75, aspect=0, near=0.1, far=1000)

            app.camera.pos.x = 0
            app.camera.pos.y = -20
            app.camera.pos.z = 20
            app.camera.look_at(ORIGIN)

            # start rendering the scene and camera
            app.renderer.render(scene, app.camera)
            app.renderer.bind(size=util.adjust_aspect)

            layout.add_widget(app.renderer)
            Clock.schedule_interval(util.rotate_cube, .01)

        self.add_widget(layout)

    def initialize_robot(self):
        if self.num_robot == 1:
            for i in range(1):
                pose, addr = self.s.recvfrom(1024)
                pose = pose.decode('utf-8').split(',')
                x, y, z = 0.0, 0.0, 1.0
                self.robot_list.append(Robot(str(i), x, y, z, 0.1, 0.1, 0.1, UAV))
        else:
            self.robot_list.append(Robot(str(0), 0.0, -1.0, 1.0, 0.1, 0.1, 0.1, UAV))
            self.robot_list.append(Robot(str(1), 0.0, 0.0, 1.0, 0.1, 0.1, 0.1, UAV))
            self.robot_list.append(Robot(str(2), 0.0, 1.0, 1.0, 0.1, 0.1, 0.1, UAV))
            self.robot_list.append(Robot(str(3), 0.0, -2.0, 1.0, 0.1, 0.1, 0.1, UAV))
            self.robot_list.append(Robot(str(4), 0.0, 2.0, 1.0, 0.1, 0.1, 0.1, UAV))

    def __init__(self, **kwargs):
        super(MainLayout, self).__init__(**kwargs)

        # get the running application
        app = App.get_running_app()

        # Initialize variables
        #
        # mode: indicate the current mode of the interface
        # message: the message to be printed out at the top of the screen
        # robots_selected: a list of robots selected by the users
        # mouse_points: a list that tracks the mouse position in ALL modes
        # tracer: a Tracer3 instance that tracks the mouse position only in GESTURE mode
        #
        self.mode = SELECTION_MODE
        self.message = ""
        self.robots_selected = []
        self.mouse_points = []
        self.robots_ids = []

        initialize_connect()
        self.s = s
        self.tracer = Tracer3(self.s)
        self.robot_list = []

        self.s.send("c".encode('utf-8'))

        data, addr = self.s.recvfrom(1024)
        self.num_robot = int(data.decode('utf-8'))

        self.initialize_robot()
        self.initialize_display(app)

    def on_touch_down(self, touch):
        with self.canvas.after:
            Color(1.0, 1.0, 0.1)
            touch.ud["line"] = Line(points=(touch.x, touch.y), width=5)
        if self.mode == GESTURE_MODE and len(self.mouse_points) > 5:
            self.tracer.touch_down_update(touch.x, touch.y)

        self.mouse_points = []

        # change mode of the interface
        if touch.is_double_tap:
            if touch.x > Window.size[0] / 2:
                if self.mode == SELECTION_MODE:
                    self.mode = GESTURE_MODE
                    self.message = "Draw Gestures: "
                    self.s.send("GESTURE".encode('utf-8'))
                else:
                    self.mode = SELECTION_MODE
                    self.message = ""
                    self.s.send("SELECTION".encode('utf-8'))
                    self.de_highlight_robots(self.robots_ids)

        return super(MainLayout, self).on_touch_down(touch)

    def on_touch_move(self, touch):
        touch.ud["line"].points += (touch.x, touch.y)

        if self.mode == GESTURE_MODE and len(self.mouse_points) > 5:
            self.tracer.touch_move_update(touch.x, touch.y)

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
        touch.ud["line"].points = []
        return super(MainLayout, self).on_touch_up(touch)

    def update_robots_selected(self):
        app = App.get_running_app()
        model_view = util.kivy_matrix_to_np_matrix(app.camera.modelview_matrix, 4, 4)
        projection = util.kivy_matrix_to_np_matrix(app.camera.projection_matrix, 4, 4)
        points = []
        self.robots_selected = []

        for i in range(len(app.cube)):
            points.append(self.to_window_coordinates(np.array([app.cube[i].pos.x,
                                                               app.cube[i].pos.y,
                                                               app.cube[i].pos.z]),
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

    def adjust_img(self):
        """
        Void function to adjust the cube picture at the left top of the window
        """
        source = App.get_running_app().im.source
        if source == 'imgs/normal_box.png':
            App.get_running_app().im.source = 'imgs/left_box.png'
        elif source == 'imgs/left_box.png':
            App.get_running_app().im.source = 'imgs/back_box.png'
        elif source == 'imgs/back_box.png':
            App.get_running_app().im.source = 'imgs/right_box.png'
        elif source == 'imgs/right_box.png':
            App.get_running_app().im.source = 'imgs/top_down.png'
        elif source == 'imgs/top_down.png':
            App.get_running_app().im.source = 'imgs/normal_box.png'

    def change_view(self):
        """
        Adjust the angle of view when users press the left top button
        """
        app = App.get_running_app()
        if app.camera.pos.x == 0 and app.camera.pos.y == -20 and app.camera.pos.z == 20:
            #app.camera.rot.x += 0.01

            app.camera.pos.x = -20
            app.camera.pos.y = 0
            app.camera.pos.z = 20
            app.camera.look_at(ORIGIN)
        elif app.camera.pos.x == -20 and app.camera.pos.y == 0 and app.camera.pos.z == 20:
            app.camera.pos.x = 0
            app.camera.pos.y = 20
            app.camera.pos.z = 20
            app.camera.look_at(ORIGIN)
        elif app.camera.pos.x == 0 and app.camera.pos.y == 20 and app.camera.pos.z == 20:
            app.camera.pos.x = 20
            app.camera.pos.y = 0
            app.camera.pos.z = 20
            app.camera.look_at(ORIGIN)
        elif app.camera.pos.x == 20 and app.camera.pos.y == 0 and app.camera.pos.z == 20:
            app.camera.pos.x = 0
            app.camera.pos.y = 0
            app.camera.pos.z = 20
            app.camera.look_at(ORIGIN)
            self.tracer.top_down_view = True
        elif app.camera.pos.x == 0 and app.camera.pos.y == 0 and app.camera.pos.z == 20:
            app.camera.pos.x = 0
            app.camera.pos.y = -20
            app.camera.pos.z = 20
            app.camera.look_at(ORIGIN)
            self.tracer.top_down_view = False

    def turn_on_top_down(self):
        app = App.get_running_app()
        if self.tracer.top_down_view:
            app.camera.pos.x = 0
            app.camera.pos.y = -20
            app.camera.pos.z = 20
            app.camera.look_at(ORIGIN)
            self.tracer.top_down_view = False
            App.get_running_app().im.source = 'imgs/normal_box.png'
        else:
            app.camera.pos.x = 0
            app.camera.pos.y = 0
            app.camera.pos.z = 20
            app.camera.look_at(ORIGIN)
            self.tracer.top_down_view = True
            App.get_running_app().im.source = 'imgs/top_down.png'

    def highlight_robots(self, robots_ids):
        """
        Highlight the robots selected

        :param robots_ids: <list> a list of ids of selected robots
        """
        app = App.get_running_app()
        select = ""
        cancel = ""
        for i in range(len(self.robot_list)):
            if self.robot_list[i].id in robots_ids:
                x, y, z = app.camera.pos.x, app.camera.pos.y, app.camera.pos.z
                if app.cube[i].material.color == WHITE:
                    app.cube[i].material.color = LIGHT_YELLOW
                    select += str(i) + " "
                else:
                    app.cube[i].material.color = WHITE
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
            if self.robot_list[i].id in robots_ids:
                app.cube[i].material.color = WHITE


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
