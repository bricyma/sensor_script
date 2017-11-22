from abc import ABCMeta, abstractmethod


class Tag:
    __metaclass__ = ABCMeta

    def __init__(self, name):
        self.name = name

    @abstractmethod
    def get_json(self):
        pass


class RadioTag(Tag):

    def __init__(self, name, values):
        super(RadioTag, self).__init__(name)
        self.values = values

    def get_json(self):
        return {
            "type": "radio",
            "name": self.name,
            "values": self.values
        }


class CheckboxTag(Tag):

    def __init__(self, name, values):
        super(CheckboxTag, self).__init__(name)
        self.values = values

    def get_json(self):
        return {
            "type": "checkbox",
            "name": self.name,
            "values": self.values
        }


class TextTag(Tag):

    def __init__(self, name):
        super(TextTag, self).__init__(name)

    def get_json(self):
        return {
            "type": "text",
            "name": self.name
        }

