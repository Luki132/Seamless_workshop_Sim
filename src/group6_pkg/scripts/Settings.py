from pathlib import Path
import json


# Enhanced version of the built-in dict class that allows
# access to elements using .name syntax:
#   od["something"] == od.something
# Source: https://goodcode.io/articles/python-dict-object/
class ObjectDict(dict):
    def __getattr__(self, name):
        if name in self:
            return self[name]
        else:
            raise AttributeError("No such attribute: " + name)

    def __setattr__(self, name, value):
        self[name] = value

    def __delattr__(self, name):
        if name in self:
            del self[name]
        else:
            raise AttributeError("No such attribute: " + name)

    def init_from_dict(self, d:dict):
        for k, v in d.items():
            if isinstance(v, dict):
                self[k] = ObjectDict()
                self[k].init_from_dict(v)
            else:
                self[k] = v

    def update_recursively(self, d:dict):
        for sk, sv in d.items():
            if sk in self and isinstance(self[sk], dict):
                if not isinstance(self[sk], ObjectDict):
                    od = ObjectDict()
                    od.init_from_dict(self[sk])
                    self[sk] = od
                self[sk].update_recursively(sv)
            else:
                self[sk] = sv


def load_settings():
    global settings

    with open(settings_source) as f:
        settings.init_from_dict(json.load(f))

    # Overwrite default settings with more specific ones
    if settings.workstation in settings:
        # Specific settings for the selected workstation available
        settings.default.update_recursively(settings[settings.workstation])

    if settings.print_after_loading:
        print(json.dumps(settings.default, indent=2))

    settings.default.workstation = settings.workstation
    settings = settings.default


settings_source = Path("../param/group6_config.json").resolve()
settings = ObjectDict()
load_settings()
