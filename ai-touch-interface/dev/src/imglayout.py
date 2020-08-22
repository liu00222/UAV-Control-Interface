from kivy.graphics import *
from kivy.uix.floatlayout import FloatLayout


class Imglayout(FloatLayout):
    def __init__(self, **args):
        super(Imglayout, self).__init__(**args)

        with self.canvas.before:
            Color(0, 0, 0, 0)
            self.rect = Rectangle(size=self.size, pos=self.pos)

        self.bind(size=self.updates, pos=self.updates)

    def updates(self, instance, value):
        self.rect.size = instance.size
        self.rect.pos = instance.pos
