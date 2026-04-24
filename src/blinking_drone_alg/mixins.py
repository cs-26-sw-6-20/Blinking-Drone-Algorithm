from dataclasses import dataclass, replace, fields


class WithMixin:
    def with_(self, **kwargs):
        """
        Mutates specific fields of a copy of the object.
        :param kwargs: The field changes of the object.
        :return: The new object with overridden fields.
        """
        return replace(self, **kwargs)

    def as_(self, cls, **kwargs):
        """
        Mutates specific fields of a copy of the object, into a new class with similar fields.
        :param cls: The target class
        :param kwargs: The fields changes of the target class
        :return: The new object with overridden fields.
        """
        existing = {f.name: getattr(self, f.name) for f in fields(self)}
        return cls(**{**existing, **kwargs})