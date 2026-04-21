from dataclasses import dataclass, replace

class WithMixin:
    def with_(self, **kwargs):
        return replace(self, **kwargs)