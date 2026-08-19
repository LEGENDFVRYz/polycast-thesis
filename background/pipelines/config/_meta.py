"""Field-level annotation shared by the config modules."""

from dataclasses import field


def inert(reason: str, **kwargs):
    """
    Mark a field whose reader is unreachable under the shipped default.

    Not dead - each has a live consumer - but the branch that reads it does not
    run, so editing it changes nothing. `reason` is what would have to change.
    Metadata only; the runtime value is untouched.
    """

    metadata = dict(kwargs.pop('metadata', {}))
    metadata['inert'] = reason
    return field(metadata=metadata, **kwargs)
