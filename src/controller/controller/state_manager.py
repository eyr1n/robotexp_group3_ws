from typing import Callable


class StateManager:
    def __init__(self):
        self.states = {}
        self.prev = ""
        self.next = ""

    def add(self, name: str, handler: Callable[[], None]):
        self.states[name] = handler

    def change(self, name: str):
        self.next = name
        
    def run(self):
        if self.next != self.prev:
            self.states[self.next]()
            self.prev = self.next
