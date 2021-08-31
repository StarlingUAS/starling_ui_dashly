import dataclasses
from typing import *
import functools
import datetime

# Helper methods for registering callbacks
@dataclasses.dataclass
class CallbackMethod:
    args: List
    kwargs: Dict
    method: Callable

    def __call__(self, *args, **kwargs):
        return self.method(*args, **kwargs)

def app_callback(*args, **kwargs):
    def decorator(method):
        return CallbackMethod(args, kwargs, method)
    return decorator


def register_callbacks(cls, app):
    # Register Callbacks as actual app callbacks
    for k, v in vars(cls.__class__).items():
        if isinstance(v, CallbackMethod):
            app.callback(*v.args, **v.kwargs)(functools.partial(v, cls))


def get_time(format="%H:%M:%S"):
    x = datetime.datetime.now()
    return x.strftime(format)


class Dashboard_Component():

    def generate_layout(self):
        return {}