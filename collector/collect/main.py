from kivy.core.window import Window

from src.collector import Collector

if __name__ == "__main__":
    # set the window size and run the main program
    # Window.size = (960, 540)
    Collector().run()
