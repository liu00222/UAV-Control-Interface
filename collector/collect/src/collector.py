# import packages
# import built-in packages
from os.path import join, dirname, abspath

# import kivy
from kivy.app import App
from kivy.graphics import *
from kivy.uix.floatlayout import FloatLayout

from src.tracer import Tracer3



class MainLayout(FloatLayout):
    def __init__(self, **kwargs):
        super(MainLayout, self).__init__(**kwargs)
        self.tracer = Tracer3()

    def on_touch_down(self, touch):
        with self.canvas.after:
            Color(0, 0, 1)
            touch.ud["line"] = Line(points=(touch.x, touch.y), width=5)
        self.tracer.touch_down_update(touch.x, touch.y)
        return super(MainLayout, self).on_touch_down(touch)

    def on_touch_move(self, touch):
        touch.ud["line"].points += (touch.x, touch.y)
        self.tracer.touch_move_update(touch.x, touch.y)
        return super(MainLayout, self).on_touch_move(touch)

    def on_touch_up(self, touch):
        self.tracer.touch_up_update(touch.x, touch.y)
        touch.ud["line"].points = []
        return super(MainLayout, self).on_touch_up(touch)


class Collector(App):
    def build(self):
        return MainLayout()
