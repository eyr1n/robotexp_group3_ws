from typing import Callable


class StateManager:
    def __init__(self):
        self.states = {}
        self.current = ""

    def add(self, name: str, handler: Callable[[], None]):
        self.states[name] = handler

    def change(self, name: str):
        if name != self.current:
            self.current = name
            self.states[name]()
